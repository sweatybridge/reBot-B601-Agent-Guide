/* A 15-state error-state EKF for the camera pose in the arm base frame:
 *
 *     x = [p, v, R, b_g, b_a]
 *     error = [dp, dv, dtheta, db_g, db_a]
 *
 * The tag is assumed fixed to the arm base. Its solve is an absolute 6-DOF
 * measurement of the camera (position and orientation). The gyro predicts
 * orientation, the accelerometer predicts translation, and the two bias states
 * let the filter coast through the frames where the tag is occluded.
 *
 * Orientation is represented as a body/right error:
 *
 *     R_true = R * Exp(dtheta)
 *
 * There is no tag-difference velocity measurement: velocity is estimated by the
 * filter from position updates and acceleration. The tag is not a moving state.
 * If the tag is bumped, the base frame moved and this model no longer applies.
 *
 * This file is deliberately standalone and has no DOM dependencies. It is
 * opt-in: the phone page defaults to tag-only and uses this file only with
 * ?filter=ekf.
 */
(function (global) {
  "use strict";

  const I3 = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];

  function zeros(rows, cols) {
    const out = [];
    for (let i = 0; i < rows; i += 1) {
      const row = [];
      for (let j = 0; j < cols; j += 1) row.push(0);
      out.push(row);
    }
    return out;
  }

  function identity(n) {
    const out = zeros(n, n);
    for (let i = 0; i < n; i += 1) out[i][i] = 1;
    return out;
  }

  function mul(a, b) {
    const out = zeros(a.length, b[0].length);
    for (let i = 0; i < a.length; i += 1) {
      for (let j = 0; j < b[0].length; j += 1) {
        let sum = 0;
        for (let k = 0; k < b.length; k += 1) sum += a[i][k] * b[k][j];
        out[i][j] = sum;
      }
    }
    return out;
  }

  function transpose(a) {
    const out = zeros(a[0].length, a.length);
    for (let i = 0; i < a.length; i += 1) {
      for (let j = 0; j < a[0].length; j += 1) out[j][i] = a[i][j];
    }
    return out;
  }

  function add(a, b) {
    return a.map((row, i) => row.map((value, j) => value + b[i][j]));
  }

  function sub(a, b) {
    return a.map((row, i) => row.map((value, j) => value - b[i][j]));
  }

  function symmetrize(a) {
    for (let i = 0; i < a.length; i += 1) {
      for (let j = i + 1; j < a.length; j += 1) {
        const mean = 0.5 * (a[i][j] + a[j][i]);
        a[i][j] = mean;
        a[j][i] = mean;
      }
    }
    return a;
  }

  function dot(a, b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  }

  function norm(a) {
    return Math.hypot(a[0], a[1], a[2]);
  }

  function cross(a, b) {
    return [
      a[1] * b[2] - a[2] * b[1],
      a[2] * b[0] - a[0] * b[2],
      a[0] * b[1] - a[1] * b[0],
    ];
  }

  function matVec(a, v) {
    return [
      a[0][0] * v[0] + a[0][1] * v[1] + a[0][2] * v[2],
      a[1][0] * v[0] + a[1][1] * v[1] + a[1][2] * v[2],
      a[2][0] * v[0] + a[2][1] * v[1] + a[2][2] * v[2],
    ];
  }

  function skew(v) {
    return [[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]];
  }

  function expSO3(omega) {
    const theta = norm(omega);
    if (theta < 1e-9) return add(I3, skew(omega));
    const axis = [omega[0] / theta, omega[1] / theta, omega[2] / theta];
    const x = axis[0], y = axis[1], z = axis[2];
    const c = Math.cos(theta);
    const s = Math.sin(theta);
    const t = 1 - c;
    return [
      [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
      [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
      [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ];
  }

  function logSO3(R) {
    const cosine = Math.max(-1, Math.min(1, (R[0][0] + R[1][1] + R[2][2] - 1) / 2));
    const angle = Math.acos(cosine);
    if (angle < 1e-8) {
      return [0.5 * (R[2][1] - R[1][2]), 0.5 * (R[0][2] - R[2][0]), 0.5 * (R[1][0] - R[0][1])];
    }
    const factor = angle / (2 * Math.sin(angle));
    return [
      factor * (R[2][1] - R[1][2]),
      factor * (R[0][2] - R[2][0]),
      factor * (R[1][0] - R[0][1]),
    ];
  }

  function orthonormalize(R) {
    let x = [R[0][0], R[1][0], R[2][0]];
    let length = norm(x);
    if (length < 1e-9) return I3.map((row) => row.slice());
    x = [x[0] / length, x[1] / length, x[2] / length];
    let y = [R[0][1], R[1][1], R[2][1]];
    const projection = dot(x, y);
    y = [y[0] - projection * x[0], y[1] - projection * x[1], y[2] - projection * x[2]];
    length = norm(y);
    if (length < 1e-9) return I3.map((row) => row.slice());
    y = [y[0] / length, y[1] / length, y[2] / length];
    const z = cross(x, y);
    return [
      [x[0], y[0], z[0]],
      [x[1], y[1], z[1]],
      [x[2], y[2], z[2]],
    ];
  }

  function invertN(m) {
    const n = m.length;
    const augmented = m.map((row, i) => row.concat(identity(n)[i]));
    for (let col = 0; col < n; col += 1) {
      let pivot = col;
      for (let row = col + 1; row < n; row += 1) {
        if (Math.abs(augmented[row][col]) > Math.abs(augmented[pivot][col])) pivot = row;
      }
      if (Math.abs(augmented[pivot][col]) < 1e-12) return null;
      if (pivot !== col) {
        const temp = augmented[pivot];
        augmented[pivot] = augmented[col];
        augmented[col] = temp;
      }
      const scale = augmented[col][col];
      for (let j = 0; j < 2 * n; j += 1) augmented[col][j] /= scale;
      for (let row = 0; row < n; row += 1) {
        if (row === col) continue;
        const factor = augmented[row][col];
        if (factor === 0) continue;
        for (let j = 0; j < 2 * n; j += 1) {
          augmented[row][j] -= factor * augmented[col][j];
        }
      }
    }
    return augmented.map((row) => row.slice(n));
  }

  function quadratic(r, s) {
    let sum = 0;
    for (let i = 0; i < r.length; i += 1) {
      for (let j = 0; j < r.length; j += 1) sum += r[i] * s[i][j] * r[j];
    }
    return sum;
  }

  function parseTag(T) {
    if (!Array.isArray(T) || T.length < 4 || !Array.isArray(T[0]) || T[0].length < 4) {
      return null;
    }
    const rotation = [
      [Number(T[0][0]), Number(T[0][1]), Number(T[0][2])],
      [Number(T[1][0]), Number(T[1][1]), Number(T[1][2])],
      [Number(T[2][0]), Number(T[2][1]), Number(T[2][2])],
    ];
    const position = [Number(T[0][3]), Number(T[1][3]), Number(T[2][3])];
    if (!position.every(Number.isFinite) || !rotation.flat().every(Number.isFinite)) return null;
    return { position, rotation: orthonormalize(rotation) };
  }

  function create(options) {
    const settings = options || {};
    const positionNoise = Number(settings.positionNoise ?? 0.030); // m, trusted
    const orientationNoise = Number(settings.orientationNoise ?? 0.030); // rad
    const velocityNoise = Number(settings.velocityNoise ?? 0.15); // m/s, initial P
    const gyroBiasNoise = Number(settings.gyroBiasNoise ?? 0.020); // rad/s, initial P
    const accelBiasNoise = Number(settings.accelBiasNoise ?? 0.15); // m/s^2, initial P
    const accelNoise = Number(settings.accelNoise ?? 0.60); // m/s^2
    const gyroNoise = Number(settings.gyroNoise ?? 0.030); // rad/s
    const gyroBiasWalk = Number(settings.gyroBiasWalk ?? 1e-4); // rad/s/sqrt(s)
    const accelBiasWalk = Number(settings.accelBiasWalk ?? 1e-3); // m/s^2/sqrt(s)
    const gate = Number(settings.gate ?? 16.81); // chi-square, 6 dof, 99%
    const resetAfter = Math.max(1, Number(settings.resetAfter ?? 3));

    let p = null;
    let v = [0, 0, 0];
    let R = null;
    let bg = [0, 0, 0];
    let ba = [0, 0, 0];
    let covariance = null;
    let initialized = false;
    let rejectStreak = 0;
    let last = { accepted: false, initialized: false, d2: null };

    function initialCovariance() {
      const P = zeros(15, 15);
      for (let i = 0; i < 3; i += 1) {
        P[i][i] = positionNoise * positionNoise;
        P[3 + i][3 + i] = velocityNoise * velocityNoise;
        P[6 + i][6 + i] = orientationNoise * orientationNoise;
        P[9 + i][9 + i] = gyroBiasNoise * gyroBiasNoise;
        P[12 + i][12 + i] = accelBiasNoise * accelBiasNoise;
      }
      return P;
    }

    function reset(pose, time) {
      p = pose.position.slice();
      v = [0, 0, 0];
      R = pose.rotation.map((row) => row.slice());
      if (!initialized) {
        bg = [0, 0, 0];
        ba = [0, 0, 0];
      }
      covariance = initialCovariance();
      initialized = true;
      rejectStreak = 0;
      last = { accepted: true, initialized: true, d2: 0, reset: true };
      return last;
    }

    function predict(gyro, accelCamera, dt) {
      if (!initialized || !(dt > 0) || !Number.isFinite(dt)) return;
      const step = Math.min(dt, 0.2);

      const omega = [0, 0, 0];
      if (gyro) {
        for (let i = 0; i < 3; i += 1) {
          const value = Number(gyro[i]);
          omega[i] = Number.isFinite(value) ? Math.max(-20, Math.min(20, value)) : 0;
        }
      }
      const omegaNominal = [omega[0] - bg[0], omega[1] - bg[1], omega[2] - bg[2]];

      const accelLocal = [0, 0, 0];
      if (accelCamera) {
        for (let i = 0; i < 3; i += 1) {
          const value = Number(accelCamera[i]);
          accelLocal[i] = Number.isFinite(value) ? Math.max(-20, Math.min(20, value)) : 0;
        }
      }
      const accelCorrected = [
        accelLocal[0] - ba[0],
        accelLocal[1] - ba[1],
        accelLocal[2] - ba[2],
      ];

      if (norm(omegaNominal) > 0) {
        R = orthonormalize(mul(R, expSO3([
          omegaNominal[0] * step,
          omegaNominal[1] * step,
          omegaNominal[2] * step,
        ])));
      }
      const aBase = matVec(R, accelCorrected);
      const a = aBase.map((value) => Math.max(-20, Math.min(20, value)));

      p = [
        p[0] + v[0] * step + 0.5 * a[0] * step * step,
        p[1] + v[1] * step + 0.5 * a[1] * step * step,
        p[2] + v[2] * step + 0.5 * a[2] * step * step,
      ];
      v = [v[0] + a[0] * step, v[1] + a[1] * step, v[2] + a[2] * step];
      for (let i = 0; i < 3; i += 1) {
        v[i] = Math.max(-3, Math.min(3, v[i]));
      }

      const F = identity(15);
      for (let i = 0; i < 3; i += 1) {
        F[i][3 + i] = step;
      }
      // dv = -R [a_local]x dtheta - R dba, for R_true = R Exp(dtheta)
      const localSkew = skew(accelCorrected);
      const rotatedSkew = mul(R, localSkew);
      for (let i = 0; i < 3; i += 1) {
        for (let j = 0; j < 3; j += 1) {
          F[3 + i][6 + j] = -rotatedSkew[i][j] * step;
          F[3 + i][12 + j] = -R[i][j] * step;
        }
      }
      // dtheta = Exp(-omega_nominal dt) dtheta - dt dbg
      const omegaSkew = skew(omegaNominal);
      for (let i = 0; i < 3; i += 1) {
        for (let j = 0; j < 3; j += 1) {
          F[6 + i][6 + j] = (i === j ? 1 : 0) - omegaSkew[i][j] * step;
        }
        F[6 + i][9 + i] = -step;
      }

      const Q = zeros(15, 15);
      const velocityProcess = accelNoise * step;
      const positionProcess = 0.5 * accelNoise * step * step;
      const angleProcess = gyroNoise * step;
      const gyroBiasProcess = gyroBiasWalk * Math.sqrt(step);
      const accelBiasProcess = accelBiasWalk * Math.sqrt(step);
      for (let i = 0; i < 3; i += 1) {
        Q[i][i] = positionProcess * positionProcess + 1e-12;
        Q[3 + i][3 + i] = velocityProcess * velocityProcess;
        Q[6 + i][6 + i] = angleProcess * angleProcess;
        Q[9 + i][9 + i] = gyroBiasProcess * gyroBiasProcess + 1e-14;
        Q[12 + i][12 + i] = accelBiasProcess * accelBiasProcess + 1e-14;
      }
      covariance = add(mul(mul(F, covariance), transpose(F)), Q);
      symmetrize(covariance);
    }

    function updateTag(T, time) {
      const pose = parseTag(T);
      if (!pose) return { accepted: false, initialized, d2: null, error: "bad pose" };
      if (!initialized) return reset(pose, time);

      const positionResidual = [
        pose.position[0] - p[0],
        pose.position[1] - p[1],
        pose.position[2] - p[2],
      ];
      // Body/right error: R_true = R * Exp(dtheta).
      const angleResidual = logSO3(mul(transpose(R), pose.rotation));
      const residual = positionResidual.concat(angleResidual);

      const H = zeros(6, 15);
      for (let i = 0; i < 3; i += 1) {
        H[i][i] = 1;
        H[3 + i][6 + i] = 1;
      }
      const PHt = zeros(15, 6);
      for (let i = 0; i < 15; i += 1) {
        for (let j = 0; j < 3; j += 1) {
          PHt[i][j] = covariance[i][j];
          PHt[i][3 + j] = covariance[i][6 + j];
        }
      }
      const HP = zeros(6, 15);
      for (let i = 0; i < 3; i += 1) {
        for (let j = 0; j < 15; j += 1) HP[i][j] = covariance[i][j];
        for (let j = 0; j < 15; j += 1) HP[3 + i][j] = covariance[6 + i][j];
      }
      const S = mul(HP, transpose(H));
      for (let i = 0; i < 6; i += 1) {
        S[i][i] += (i < 3 ? positionNoise * positionNoise : orientationNoise * orientationNoise);
      }

      const Sinv = invertN(S);
      if (!Sinv) return { accepted: false, initialized, d2: null, error: "singular S" };
      const d2 = quadratic(residual, Sinv);
      if (!Number.isFinite(d2) || d2 > gate) {
        rejectStreak += 1;
        last = { accepted: false, initialized, d2: d2 };
        if (rejectStreak >= resetAfter) {
          return reset(pose, time);
        }
        return last;
      }

      const K = mul(PHt, Sinv);
      const delta = zeros(15, 1);
      for (let i = 0; i < 15; i += 1) {
        let sum = 0;
        for (let j = 0; j < 6; j += 1) sum += K[i][j] * residual[j];
        delta[i][0] = sum;
      }
      for (let i = 0; i < 3; i += 1) {
        p[i] += delta[i][0];
        v[i] += delta[3 + i][0];
        bg[i] += delta[9 + i][0];
        ba[i] += delta[12 + i][0];
      }
      const dTheta = [delta[6][0], delta[7][0], delta[8][0]];
      if (norm(dTheta) > 0) R = orthonormalize(mul(R, expSO3(dTheta)));
      covariance = sub(covariance, mul(K, transpose(PHt)));
      symmetrize(covariance);

      rejectStreak = 0;
      last = { accepted: true, initialized: true, d2: d2 };
      return last;
    }

    function pose() {
      if (!p || !R) return null;
      return {
        position: p.slice(),
        rotation: R.map((row) => row.slice()),
      };
    }

    function matrix() {
      if (!p || !R) return null;
      return [
        [R[0][0], R[0][1], R[0][2], p[0]],
        [R[1][0], R[1][1], R[1][2], p[1]],
        [R[2][0], R[2][1], R[2][2], p[2]],
        [0, 0, 0, 1],
      ];
    }

    return {
      predict: predict,
      updateTag: updateTag,
      reset: reset,
      pose: pose,
      matrix: matrix,
      position: function () { return p ? p.slice() : null; },
      rotation: function () { return R ? R.map((row) => row.slice()) : null; },
      velocity: function () { return v.slice(); },
      biases: function () { return { gyro: bg.slice(), accel: ba.slice() }; },
      covariance: function () { return covariance; },
      get initialized() { return initialized; },
      get last() { return last; },
      get rejectionCount() { return rejectStreak; },
    };
  }

  global.RebotEkf = { create: create };
})(typeof window !== "undefined" ? window : globalThis);
