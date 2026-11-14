/* Draw the arm's reachable band on top of the camera preview.
 *
 * The video already shows the table; the only missing piece is which part of
 * that table the arm can serve. This module adds a canvas over the video and
 * paints an annulus between the arm's minimum and maximum reach, projected
 * through the same pinhole camera the planner uses.
 *
 * Everything geometric happens here, in the page:
 *
 *   * the arm's D-H-style chain from `rebot_ik.py` is transcribed below;
 *   * a damped-least-squares IK finds the radial limits of a tool-down pose;
 *   * the pinhole + Brown-Conrady projection maps the base-frame ring to pixels;
 *   * a 2D canvas draws the band, clipped to the letterboxed video rectangle.
 *
 * The server only supplies numbers it already has: the calibration (K, dist)
 * and, when a fixed-camera rig is fitted, `T_base_cam`. In handheld mode the
 * browser ArUco detector solves `T_base_cam`; between tag frames the phone gyro
 * propagates the orientation while the tag position is held. `setPose()` stays
 * for the workstation tap path.
 *
 * Why the reach band is computed rather than guessed: the interesting shape is
 * not a circle. The arm cannot fold its wrist close to its own base while
 * keeping the tool pointing down, so the annulus has a real inner edge; joint 1
 * stops 40 deg short of a full turn, so it also has a wedge-shaped gap.
 */
(function () {
  "use strict";

  /* ------------------------------------------------------------- arm model
   *
   * Transcribed from `app/rebot_ik.py`, which is itself the Seeed URDF:
   * (origin xyz, origin rpy, joint axis, lower, upper). Keep the two in sync;
   * the reach band is only as good as this chain.
   */
  const CHAIN = [
    { xyz: [-0.00034283, -0.00098683, 0.075], rpy: [0, 0, 0], axis: [0, 0, -1], lo: -2.80, hi: 2.80 },
    { xyz: [0.020343, 0.027237, 0.07], rpy: [-1.5708, 0, 0], axis: [0, 0, 1], lo: 0, hi: 3.14 },
    { xyz: [-0.236, 0, 0], rpy: [0, 0, 0], axis: [0, 0, -1], lo: 0, hi: 3.14 },
    { xyz: [0.228, -0.072746, 0.0045], rpy: [0, 0, 0], axis: [0, 0, -1], lo: -1.57, hi: 1.57 },
    { xyz: [0.087, -0.048, -0.03075], rpy: [-1.5708, 0, 0], axis: [0, 0, -1], lo: -1.57, hi: 1.57 },
    { xyz: [0.0365, 0, 0.048], rpy: [0, 1.5708, 0], axis: [0, 0, -1], lo: -3.14, hi: 3.14 },
  ];
  const TOOL_XYZ = [0, 0, 0.16621];
  const TOOL_RPY = [3.1416, -1.5708, 0];
  const LIMITS = CHAIN.map((joint) => [joint.lo, joint.hi]);

  // The planner refuses a target further than this from the base axis
  // (`--max-reach`, 0.75 m). The kinematic envelope can be a centimetre or two
  // wider; clipping to the gate keeps the overlay from promising a plan the
  // gateway will reject.
  const MAX_REACH = 0.75;

  // The tool-down pose the planner reaches with at the parked pose: a seed for
  // the IK, not a calibration. The radial scan moves away from it in 1 cm
  // steps, so every later solve starts from its neighbour.
  const SEEDS = [
    [0, 2.02, 0.52, 1.5, 0, 0],
    [0, 2.457, 1.246, 1.315, -0.376, 0.034],
    [0, 1.2, 0.2, 1.5, 0, 0],
    [0, 0, 0, 0, 0, 0],
  ];

  /* --------------------------------------------------------- 3x3 / 3-vector */
  function identity() {
    return [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
  }

  function matMul(a, b) {
    const out = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
    for (let i = 0; i < 3; i += 1) {
      for (let j = 0; j < 3; j += 1) {
        out[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
      }
    }
    return out;
  }

  function matVec(a, v) {
    return [
      a[0][0] * v[0] + a[0][1] * v[1] + a[0][2] * v[2],
      a[1][0] * v[0] + a[1][1] * v[1] + a[1][2] * v[2],
      a[2][0] * v[0] + a[2][1] * v[1] + a[2][2] * v[2],
    ];
  }

  function transpose(a) {
    return [[a[0][0], a[1][0], a[2][0]], [a[0][1], a[1][1], a[2][1]], [a[0][2], a[1][2], a[2][2]]];
  }

  function rpyToMatrix(rpy) {
    const [roll, pitch, yaw] = rpy;
    const cr = Math.cos(roll), sr = Math.sin(roll);
    const cp = Math.cos(pitch), sp = Math.sin(pitch);
    const cy = Math.cos(yaw), sy = Math.sin(yaw);
    const rx = [[1, 0, 0], [0, cr, -sr], [0, sr, cr]];
    const ry = [[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]];
    const rz = [[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]];
    return matMul(matMul(rz, ry), rx);
  }

  function axisAngle(axis, angle) {
    const [x, y, z] = axis;
    const c = Math.cos(angle), s = Math.sin(angle), t = 1 - c;
    return [
      [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
      [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
      [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ];
  }

  function rotationError(current, target) {
    // Axis-angle of `target @ current^T`, the same residual rebot_ik uses.
    const error = matMul(target, transpose(current));
    const cosine = Math.max(-1, Math.min(1, (error[0][0] + error[1][1] + error[2][2] - 1) / 2));
    const angle = Math.acos(cosine);
    if (angle < 1e-9) return [0, 0, 0];
    const axis = [
      (error[2][1] - error[1][2]) / (2 * Math.sin(angle)),
      (error[0][2] - error[2][0]) / (2 * Math.sin(angle)),
      (error[1][0] - error[0][1]) / (2 * Math.sin(angle)),
    ];
    return [axis[0] * angle, axis[1] * angle, axis[2] * angle];
  }

  function rotationFromPose(T) {
    return [[T[0][0], T[0][1], T[0][2]],
            [T[1][0], T[1][1], T[1][2]],
            [T[2][0], T[2][1], T[2][2]]];
  }

  function blendOrientation(current, target, fraction) {
    // Shortest-path correction toward the tag, so a noisy 3 Hz pose does not
    // snap the band. `rotationError` is the axis-angle of target * current^T.
    const error = rotationError(current, target);
    const angle = Math.hypot(error[0], error[1], error[2]);
    if (!(angle > 1e-9) || !Number.isFinite(angle)) return current;
    const axis = [error[0] / angle, error[1] / angle, error[2] / angle];
    return matMul(axisAngle(axis, angle * fraction), current);
  }

  function forward(q, withFrames) {
    let rotation = identity();
    let position = [0, 0, 0];
    const frames = [];
    for (let index = 0; index < CHAIN.length; index += 1) {
      const joint = CHAIN[index];
      const offset = matVec(rotation, joint.xyz);
      position = [position[0] + offset[0], position[1] + offset[1], position[2] + offset[2]];
      rotation = matMul(rotation, rpyToMatrix(joint.rpy));
      if (withFrames) {
        frames.push({ position: position.slice(), axis: matVec(rotation, joint.axis) });
      }
      rotation = matMul(rotation, axisAngle(joint.axis, q[index]));
    }
    const toolOffset = matVec(rotation, TOOL_XYZ);
    position = [position[0] + toolOffset[0], position[1] + toolOffset[1], position[2] + toolOffset[2]];
    rotation = matMul(rotation, rpyToMatrix(TOOL_RPY));
    return withFrames ? { rotation: rotation, position: position, frames: frames }
      : { rotation: rotation, position: position };
  }

  function withinLimits(q, margin) {
    const inset = margin || 0;
    for (let index = 0; index < LIMITS.length; index += 1) {
      if (q[index] < LIMITS[index][0] + inset || q[index] > LIMITS[index][1] - inset) return false;
    }
    return true;
  }

  /* --------------------------------------------------------------- 6x6 solve */
  function solve6(a, b) {
    // Gauss-Jordan with partial pivoting; a is 6x6 row-major, b is 6x1.
    const m = a.map((row, index) => row.slice().concat([b[index]]));
    for (let column = 0; column < 6; column += 1) {
      let pivot = column;
      for (let row = column + 1; row < 6; row += 1) {
        if (Math.abs(m[row][column]) > Math.abs(m[pivot][column])) pivot = row;
      }
      if (Math.abs(m[pivot][column]) < 1e-12) return null;
      if (pivot !== column) {
        const swap = m[pivot];
        m[pivot] = m[column];
        m[column] = swap;
      }
      const divisor = m[column][column];
      for (let k = column; k < 7; k += 1) m[column][k] /= divisor;
      for (let row = 0; row < 6; row += 1) {
        if (row === column) continue;
        const factor = m[row][column];
        if (!factor) continue;
        for (let k = column; k < 7; k += 1) m[row][k] -= factor * m[column][k];
      }
    }
    return m.map((row) => row[6]);
  }

  /* ------------------------------------------------------------------- IK
   *
   * The same damped least squares as `rebot_ik.ik()`: position plus the last
   * three orientation rows, weighted so radians and metres are comparable. It
   * only has to answer "is a tool-down pose at (r, 0, z) inside the limits?",
   * and it is seeded from the previous radius, so a few iterations are enough.
   */
  function inverseKinematics(targetPosition, targetRotation, seed) {
    const tolPosition = 1e-3;
    const tolRotation = 2e-3;
    const damping = 0.02;
    const rotWeight = 0.5;
    const maxStep = 0.15;
    const low = LIMITS.map((range, index) => Math.min(range[0], seed[index]));
    const high = LIMITS.map((range, index) => Math.max(range[1], seed[index]));
    let q = seed.slice();
    let last = { ok: false, posErr: 1, rotErr: 1 };

    for (let iteration = 0; iteration < 160; iteration += 1) {
      const current = forward(q, true);
      const posErr = [
        targetPosition[0] - current.position[0],
        targetPosition[1] - current.position[1],
        targetPosition[2] - current.position[2],
      ];
      const rotErr = rotationError(current.rotation, targetRotation);
      const positionNorm = Math.hypot(posErr[0], posErr[1], posErr[2]);
      const rotationNorm = Math.hypot(rotErr[0], rotErr[1], rotErr[2]);
      last = { ok: false, posErr: positionNorm, rotErr: rotationNorm };
      if (positionNorm < tolPosition && rotationNorm < tolRotation) {
        last.ok = true;
        return { q: q, info: last };
      }

      // Geometric Jacobian, rows 0-2 linear and 3-5 angular.
      const jacobian = [];
      for (let row = 0; row < 6; row += 1) jacobian.push(new Array(6).fill(0));
      for (let joint = 0; joint < 6; joint += 1) {
        const frame = current.frames[joint];
        const dx = current.position[0] - frame.position[0];
        const dy = current.position[1] - frame.position[1];
        const dz = current.position[2] - frame.position[2];
        jacobian[0][joint] = frame.axis[1] * dz - frame.axis[2] * dy;
        jacobian[1][joint] = frame.axis[2] * dx - frame.axis[0] * dz;
        jacobian[2][joint] = frame.axis[0] * dy - frame.axis[1] * dx;
        jacobian[3][joint] = frame.axis[0];
        jacobian[4][joint] = frame.axis[1];
        jacobian[5][joint] = frame.axis[2];
      }
      for (let row = 3; row < 6; row += 1) {
        for (let joint = 0; joint < 6; joint += 1) jacobian[row][joint] *= rotWeight;
      }

      const system = [];
      for (let row = 0; row < 6; row += 1) {
        system.push(new Array(6).fill(0));
        for (let column = 0; column < 6; column += 1) {
          let value = 0;
          for (let k = 0; k < 6; k += 1) value += jacobian[row][k] * jacobian[column][k];
          system[row][column] = value + (row === column ? damping * damping : 0);
        }
      }
      const errorVector = posErr.concat(rotErr.map((value) => value * rotWeight));
      const solved = solve6(system, errorVector);
      if (!solved) return { q: q, info: last };

      let step = new Array(6).fill(0);
      for (let joint = 0; joint < 6; joint += 1) {
        for (let row = 0; row < 6; row += 1) step[joint] += jacobian[row][joint] * solved[row];
      }
      const largest = Math.max.apply(null, step.map(Math.abs));
      if (largest > maxStep) {
        for (let joint = 0; joint < 6; joint += 1) step[joint] *= maxStep / largest;
      }

      const previous = positionNorm + rotWeight * rotationNorm;
      for (let attempt = 0; attempt < 6; attempt += 1) {
        const candidate = q.map((value, index) => Math.max(low[index], Math.min(high[index], value + step[index])));
        const trial = forward(candidate);
        const trialPosition = Math.hypot(targetPosition[0] - trial.position[0],
          targetPosition[1] - trial.position[1], targetPosition[2] - trial.position[2]);
        const trialRotation = rotationError(trial.rotation, targetRotation);
        const trialError = trialPosition + rotWeight
          * Math.hypot(trialRotation[0], trialRotation[1], trialRotation[2]);
        if (trialError < previous || attempt === 5) {
          q = candidate;
          break;
        }
        for (let joint = 0; joint < 6; joint += 1) step[joint] *= 0.5;
      }
    }
    return { q: q, info: last };
  }

  function solveAt(radius, flangeZ, seed) {
    const target = [radius, 0, flangeZ];
    // Tool pointing straight down: the parked arm's orientation, and the one
    // `plan_to_target.work_orientation` settles on for a table target.
    const attempt = inverseKinematics(target, identity(), seed);
    if (!attempt.info.ok || !withinLimits(attempt.q)) return null;
    const actual = forward(attempt.q);
    if (Math.hypot(actual.position[0] - radius, actual.position[1], actual.position[2] - flangeZ) > 0.003) {
      return null;
    }
    return attempt.q;
  }

  /* ------------------------------------------------------------- reach band
   *
   * The arm is rotationally symmetric about base z once the wrist yaw follows
   * the direction it reaches in, so one radial scan at +x describes every
   * angle. What is *not* symmetric is joint 1's limits: +-2.8 rad leaves a
   * 40 deg wedge behind the base that no tool-down pose can enter.
   *
   * `flangeZ` is where the *flange* has to be. The planner aims the flange
   * `tip_m` above the point it wants the tip at, and then `hover_m` up for a
   * tap, so a table point (x, y, plane_z) becomes a flange at
   * plane_z + hover_m + tip_m.
   */
  function computeReachBand(planeZ, hoverM, tipM) {
    const flangeZ = planeZ + hoverM + tipM;
    const step = 0.01;

    function reachable(radius, seed) {
      let q = solveAt(radius, flangeZ, seed);
      if (q) return q;
      for (let index = 0; index < SEEDS.length; index += 1) {
        q = solveAt(radius, flangeZ, SEEDS[index]);
        if (q) return q;
      }
      return null;
    }

    // Find one point inside the band first: 0.60 m is a normal working radius
    // on this arm, and the scan then walks outwards and inwards from there.
    let anchor = null;
    let seed = SEEDS[0];
    const anchors = [0.60, 0.50, 0.65, 0.45, 0.70, 0.40, 0.30, 0.20];
    for (let index = 0; index < anchors.length && !anchor; index += 1) {
      const q = reachable(anchors[index], seed);
      if (q) {
        anchor = anchors[index];
        seed = q;
      }
    }
    if (anchor === null) return null;

    let outer = anchor;
    let outerSeed = seed;
    for (let radius = anchor + step; radius <= 0.90; radius += step) {
      const q = reachable(radius, outerSeed);
      if (!q) break;
      outer = radius;
      outerSeed = q;
    }

    let inner = anchor;
    let innerSeed = seed;
    for (let radius = anchor - step; radius >= 0.02; radius -= step) {
      const q = reachable(radius, innerSeed);
      if (!q) break;
      inner = radius;
      innerSeed = q;
    }

    return {
      inner: Math.max(0, inner),
      outer: Math.min(outer, MAX_REACH),
      // The reachable azimuths are joint 1's range; the gap behind the base is
      // the part of the circle it cannot turn into.
      sector: Math.min(Math.abs(LIMITS[0][0]), Math.abs(LIMITS[0][1])),
      tipHeight: planeZ + hoverM,
      flangeZ: flangeZ,
    };
  }

  /* ------------------------------------------------------------ projection */
  function makeCamera(calibration, videoWidth, videoHeight) {
    const source = calibration.image_size;
    const sx = videoWidth / source[0];
    const sy = videoHeight / source[1];
    if (Math.abs(sx - sy) / Math.max(sx, sy) > 0.01) {
      throw new Error("calibration is for " + source[0] + "x" + source[1]
        + " but the camera is " + videoWidth + "x" + videoHeight
        + "; recalibrate at this size (the planner would refuse it too)");
    }
    const K = calibration.K;
    const dist = calibration.dist || [];
    return {
      fx: K[0][0] * sx, fy: K[1][1] * sy,
      cx: K[0][2] * sx, cy: K[1][2] * sy,
      k1: dist[0] || 0, k2: dist[1] || 0,
      p1: dist[2] || 0, p2: dist[3] || 0, k3: dist[4] || 0,
      source: source,
    };
  }

  function project(camera, pose, point) {
    // T_base<-cam: bring the base point into the camera, then distort and
    // project exactly as `cv2.projectPoints` does.
    const rotation = pose.rotation;
    const translation = pose.translation;
    const dx = point[0] - translation[0];
    const dy = point[1] - translation[1];
    const dz = point[2] - translation[2];
    // `rotation` is T_base<-cam, so the camera sees R^T (p - t).
    const x = rotation[0][0] * dx + rotation[1][0] * dy + rotation[2][0] * dz;
    const y = rotation[0][1] * dx + rotation[1][1] * dy + rotation[2][1] * dz;
    const z = rotation[0][2] * dx + rotation[1][2] * dy + rotation[2][2] * dz;
    if (z <= 1e-6) return null;
    const xn = x / z;
    const yn = y / z;
    const r2 = xn * xn + yn * yn;
    const radial = 1 + camera.k1 * r2 + camera.k2 * r2 * r2 + camera.k3 * r2 * r2 * r2;
    const xd = xn * radial + 2 * camera.p1 * xn * yn + camera.p2 * (r2 + 2 * xn * xn);
    const yd = yn * radial + camera.p1 * (r2 + 2 * yn * yn) + 2 * camera.p2 * xn * yn;
    return { u: camera.fx * xd + camera.cx, v: camera.fy * yd + camera.cy };
  }

  function matrixPose(T) {
    return {
      rotation: [[T[0][0], T[0][1], T[0][2]], [T[1][0], T[1][1], T[1][2]], [T[2][0], T[2][1], T[2][2]]],
      translation: [T[0][3], T[1][3], T[2][3]],
    };
  }

  /* ---------------------------------------------------------------- drawing */
  const COLOR = "69, 201, 138";           // the page's --good, as rgba components

  function create(options) {
    const video = options.video;
    const canvas = options.canvas;
    const status = options.onStatus || function () {};
    const token = options.token || (() => "");
    const context = canvas.getContext("2d");

    let visible = false;
    let loading = false;
    let config = null;              // /reach.json
    let band = null;                // computed annulus, cached by flange height
    let bandKey = "";
    let lastPose = null;            // T_base_cam from the last plan (handheld)
    let lastPoseLabel = "";
    let lastError = "";
    let detector = null;            // browser ArUco, handheld only
    let livePose = null;            // T_base_cam from the base tag, this frame
    let liveAt = 0;
    let detectTimer = null;
    let detectMisses = 0;
    let lossLogged = false;
    let baseCamera = null;          // R_base_cam propagated by the phone gyro
    let motionStarted = false;
    let motionSeen = false;
    let motionPermission = "unknown";
    let motionRequest = null;
    let motionLogged = false;
    let motionStatusLogged = false;
    let lastMotionAt = 0;
    let animationId = 0;

    function headers() {
      const value = typeof token === "function" ? token() : token;
      return value ? { "X-Rebot-Token": value } : {};
    }

    async function load() {
      const response = await fetch("/reach.json", { headers: headers(), cache: "no-store" });
      const data = await response.json().catch(() => ({}));
      if (!response.ok || !data.ok) {
        throw new Error(data.error || ("the host answered " + response.status));
      }
      config = data;
      const key = [data.plane_z, data.hover_m, data.tip_m].join("|");
      if (bandKey !== key || !band) {
        band = computeReachBand(Number(data.plane_z), Number(data.hover_m), Number(data.tip_m));
        bandKey = key;
      }
      if (!band) throw new Error("the arm model has no reachable band at this height");
      return data;
    }

    function sizeCanvas() {
      const rect = canvas.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      const width = Math.max(1, Math.round(rect.width * dpr));
      const height = Math.max(1, Math.round(rect.height * dpr));
      if (canvas.width !== width || canvas.height !== height) {
        canvas.width = width;
        canvas.height = height;
      }
      context.setTransform(dpr, 0, 0, dpr, 0, 0);
      return { width: rect.width, height: rect.height };
    }

    function wrapText(text, maxWidth) {
      const words = String(text).split(/\s+/);
      const lines = [];
      let line = "";
      for (const word of words) {
        const candidate = line ? line + " " + word : word;
        if (line && context.measureText(candidate).width > maxWidth) {
          lines.push(line);
          line = word;
        } else {
          line = candidate;
        }
      }
      if (line) lines.push(line);
      return lines;
    }

    function drawEmpty(message) {
      const size = sizeCanvas();
      context.clearRect(0, 0, size.width, size.height);
      if (!message) return;
      const box = contentBox();
      const left = box ? box.left : 0;
      const maxWidth = Math.max(120, (box ? box.width : size.width) - 24);
      context.font = "500 12.5px system-ui, -apple-system, Segoe UI, Roboto, sans-serif";
      const lines = wrapText(message, maxWidth);
      const textWidth = Math.min(maxWidth,
        Math.max.apply(null, lines.map((line) => context.measureText(line).width)));
      const x = left + 12;
      const y = 12;
      const height = lines.length * 17 + 14;
      context.fillStyle = "rgba(16, 20, 26, 0.86)";
      context.strokeStyle = "rgba(240, 180, 41, 0.65)";
      context.lineWidth = 1;
      if (context.roundRect) {
        context.beginPath();
        context.roundRect(x, y, textWidth + 22, height, 9);
        context.fill();
        context.stroke();
      } else {
        context.fillRect(x, y, textWidth + 22, height);
      }
      context.fillStyle = "#f0b429";
      lines.forEach((line, index) => context.fillText(line, x + 11, y + 21 + index * 17));
    }

    function contentBox() {
      const rect = canvas.getBoundingClientRect();
      const width = video.videoWidth;
      const height = video.videoHeight;
      if (!width || !height) return null;
      const scale = Math.min(rect.width / width, rect.height / height);
      return {
        left: (rect.width - width * scale) / 2,
        top: (rect.height - height * scale) / 2,
        width: width * scale,
        height: height * scale,
        scale: scale,
      };
    }

    function toCanvas(point, box) {
      // `project` answers in video pixels; the canvas draws in CSS pixels.
      return point ? { u: box.left + point.u * box.scale, v: box.top + point.v * box.scale } : null;
    }

    function ringPoint(camera, pose, radius, angle, box) {
      return toCanvas(project(camera, pose,
        [radius * Math.cos(angle), radius * Math.sin(angle), band.tipHeight]), box);
    }

    function drawBand(camera, pose, label) {
      const size = sizeCanvas();
      context.clearRect(0, 0, size.width, size.height);
      const box = contentBox();
      if (!box) return;
      context.save();
      context.beginPath();
      context.rect(box.left, box.top, box.width, box.height);
      context.clip();

      const samples = 128;
      const sector = band.sector;
      const outer = [];
      const inner = [];
      for (let index = 0; index <= samples; index += 1) {
        const angle = -sector + (2 * sector * index) / samples;
        const far = ringPoint(camera, pose, band.outer, angle, box);
        const near = ringPoint(camera, pose, band.inner, angle, box);
        if (far) outer.push(far);
        if (near) inner.push(near);
      }

      // The band itself: outer arc forwards, inner arc backwards, filled once.
      if (outer.length > 1 && inner.length > 1) {
        context.beginPath();
        context.moveTo(outer[0].u, outer[0].v);
        for (let index = 1; index < outer.length; index += 1) {
          context.lineTo(outer[index].u, outer[index].v);
        }
        for (let index = inner.length - 1; index >= 0; index -= 1) {
          context.lineTo(inner[index].u, inner[index].v);
        }
        context.closePath();
        context.fillStyle = "rgba(" + COLOR + ", 0.14)";
        context.fill();
      }

      context.setLineDash([7, 5]);
      context.lineWidth = 2;
      context.strokeStyle = "rgba(" + COLOR + ", 0.95)";
      for (const ring of [outer, inner]) {
        if (ring.length < 2) continue;
        context.beginPath();
        context.moveTo(ring[0].u, ring[0].v);
        for (let index = 1; index < ring.length; index += 1) {
          context.lineTo(ring[index].u, ring[index].v);
        }
        context.stroke();
      }
      context.setLineDash([]);

      // The base, so the band has an obvious centre even when it is off-screen.
      const base = toCanvas(project(camera, pose, [0, 0, band.tipHeight]), box);
      if (base) {
        context.beginPath();
        context.arc(base.u, base.v, 4, 0, Math.PI * 2);
        context.fillStyle = "rgba(" + COLOR + ", 0.95)";
        context.fill();
      }

      // A two-line legend in the top-left of the *video*, not of the letterbox.
      // Two lines because a phone held in landscape letterboxes the preview to
      // a narrow strip, and one long line would be cut off by the clip.
      const first = "arm reach · " + band.inner.toFixed(2) + "–" + band.outer.toFixed(2) + " m";
      const second = "tip " + Math.round((band.tipHeight - Number(config.plane_z)) * 1000)
        + " mm up" + (label ? " · " + label : "");
      context.font = "500 12.5px system-ui, -apple-system, Segoe UI, Roboto, sans-serif";
      const textWidth = Math.max(context.measureText(first).width,
        context.measureText(second).width);
      const x = box.left + 12;
      const y = box.top + 12;
      context.fillStyle = "rgba(16, 20, 26, 0.86)";
      context.strokeStyle = "rgba(" + COLOR + ", 0.5)";
      context.lineWidth = 1;
      if (context.roundRect) {
        context.beginPath();
        context.roundRect(x, y, textWidth + 22, 46, 9);
        context.fill();
        context.stroke();
      } else {
        context.fillRect(x, y, textWidth + 22, 46);
      }
      context.fillStyle = "#cfe9dd";
      context.fillText(first, x + 11, y + 18);
      context.fillStyle = "#95c9b0";
      context.fillText(second, x + 11, y + 36);
      context.restore();
    }

    function clearCanvas() {
      const size = sizeCanvas();
      context.clearRect(0, 0, size.width, size.height);
    }

    function noteMotion(message) {
      if (motionStatusLogged) return;
      motionStatusLogged = true;
      status(message, "warn");
    }

    function requestMotionPermission() {
      if (motionPermission !== "unknown") return;
      if (motionRequest) return;
      if (!window.DeviceMotionEvent) {
        motionPermission = "unsupported";
        noteMotion("reach overlay: no motion sensor; the band updates when the tag is seen");
        return;
      }
      // iOS gates the gyro behind a permission prompt that must be requested
      // from a user gesture. toggle() calls this before awaiting /reach.json.
      if (typeof window.DeviceMotionEvent.requestPermission !== "function") {
        motionPermission = "granted";
        return;
      }
      try {
        motionRequest = window.DeviceMotionEvent.requestPermission()
          .then((result) => {
            motionPermission = result === "granted" ? "granted" : "denied";
            if (motionPermission === "granted") {
              startMotion();
            } else {
              noteMotion("reach overlay: motion permission denied; the band updates when the tag is seen");
            }
          })
          .catch(() => {
            motionPermission = "denied";
            noteMotion("reach overlay: motion permission denied; the band updates when the tag is seen");
          });
      } catch (error) {
        motionPermission = "denied";
        noteMotion("reach overlay: motion permission denied; the band updates when the tag is seen");
      }
    }

    function startMotion() {
      if (motionStarted || !visible || !config || config.T_base_cam) return;
      if (motionPermission !== "granted") return;
      window.addEventListener("devicemotion", onMotion);
      motionStarted = true;
      motionSeen = false;
      lastMotionAt = 0;
    }

    function stopMotion() {
      if (motionStarted) {
        window.removeEventListener("devicemotion", onMotion);
        motionStarted = false;
      }
      motionSeen = false;
      lastMotionAt = 0;
      baseCamera = null;
    }

    function onMotion(event) {
      if (!visible || !motionStarted || !livePose || !baseCamera) return;
      const rate = event.rotationRate;
      if (!rate) return;
      const alpha = Number(rate.alpha);
      const beta = Number(rate.beta);
      const gamma = Number(rate.gamma);
      if (!Number.isFinite(alpha) || !Number.isFinite(beta) || !Number.isFinite(gamma)) {
        return;
      }
      // performance.now() keeps the integration on one clock across Safari and
      // Chrome; the event's own timeStamp origin differs between them.
      const now = performance.now();
      if (!motionSeen) {
        motionSeen = true;
        if (!motionLogged) {
          motionLogged = true;
          status("reach overlay: phone motion is smoothing the band", "good");
        }
        if (livePose) startAnimation();
      }
      if (!lastMotionAt) {
        lastMotionAt = now;
        return;
      }
      const dt = (now - lastMotionAt) / 1000;
      lastMotionAt = now;
      if (!(dt > 0 && dt < 0.2)) return;

      // DeviceMotion: alpha is about z, beta about x, gamma about y. The rear
      // camera has x_cam = x_device, y_cam = -y_device, z_cam = -z_device.
      const omega = [beta, -gamma, -alpha].map((value) => value * Math.PI / 180);
      const magnitude = Math.hypot(omega[0], omega[1], omega[2]);
      if (magnitude < 1e-6) return;
      const axis = [omega[0] / magnitude, omega[1] / magnitude, omega[2] / magnitude];
      baseCamera = matMul(baseCamera, axisAngle(axis, magnitude * dt));
    }

    function applyTagOrientation(T) {
      const target = rotationFromPose(T);
      if (!baseCamera || !motionSeen) {
        // No gyro samples: follow the tag directly, as the old overlay did.
        baseCamera = target;
      } else {
        baseCamera = blendOrientation(baseCamera, target, 0.6);
      }
      lastMotionAt = performance.now();
    }

    function predictedPose() {
      if (!livePose) return null;
      if (!baseCamera) return livePose;
      const R = baseCamera;
      return [
        [R[0][0], R[0][1], R[0][2], livePose[0][3]],
        [R[1][0], R[1][1], R[1][2], livePose[1][3]],
        [R[2][0], R[2][1], R[2][2], livePose[2][3]],
        [0, 0, 0, 1],
      ];
    }

    function stopAnimation() {
      if (animationId) {
        cancelAnimationFrame(animationId);
        animationId = 0;
      }
    }

    function animate() {
      animationId = 0;
      if (!visible || !config || config.T_base_cam) return;
      if (livePose && Date.now() - liveAt < 3000) {
        draw();
        animationId = requestAnimationFrame(animate);
      }
    }

    function startAnimation() {
      if (animationId || !visible || !config || config.T_base_cam || !motionSeen) return;
      animationId = requestAnimationFrame(animate);
    }

    function stopDetection() {
      if (detectTimer) {
        clearTimeout(detectTimer);
        detectTimer = null;
      }
      stopMotion();
      stopAnimation();
    }

    function startDetection() {
      if (!visible || !config || config.T_base_cam) return;
      if (!config.marker || !window.ArucoPose) return;
      if (config.marker.id !== 0 || (config.marker.dictionary
        && config.marker.dictionary !== "4x4_50")) {
        lastError = "browser pose needs a 4x4_50 id 0 tag; fit a fixed rig instead";
        drawEmpty(lastError);
        return;
      }
      if (!detector) {
        detector = window.ArucoPose.create({
          video: video,
          calibration: config.calibration,
          marker: config.marker,
          planeZ: Number(config.plane_z),
        });
      }
      startMotion();
      if (livePose) startAnimation();
      if (detectTimer) {
        // The camera may have just produced its first frame after the overlay
        // was switched on; restart the grace rather than clearing immediately.
        if (!livePose) {
          liveAt = Date.now();
          detectMisses = 0;
        }
        return;
      }
      // The "few seconds" grace starts when the overlay starts looking, not at
      // some stale timestamp from a previous toggle.
      if (!livePose) liveAt = Date.now();
      lossLogged = false;

      function tick() {
        detectTimer = null;
        if (!visible || !config || config.T_base_cam) return;
        if (video.videoWidth && video.videoHeight) {
          let found = null;
          try {
            found = detector.detect();
          } catch (error) {
            lastError = error.message;
          }
          if (found && found.T_base_cam) {
            applyTagOrientation(found.T_base_cam);
            livePose = found.T_base_cam;
            liveAt = Date.now();
            detectMisses = 0;
            lossLogged = false;
            startAnimation();
            draw();
          } else {
            detectMisses += 1;
            // A single missed frame is normal (blur, a hand, a bad angle).
            // Keep the last good pose for three seconds, then clear -- a band
            // left over from a camera that moved is worse than no band.
            if (detectMisses > 2 && Date.now() - liveAt > 3000) {
              livePose = null;
              lastPose = null;
              if (!lossLogged) {
                lossLogged = true;
                status("reach overlay: base tag not seen for 3 s; band hidden", "warn");
              }
              stopAnimation();
              baseCamera = null;
              clearCanvas();
            }
          }
        }
        detectTimer = setTimeout(tick, 320);
      }
      tick();
    }

    function draw() {
      if (!visible) return;
      if (!video.videoWidth || !video.videoHeight) {
        drawEmpty("start the camera first");
        return;
      }
      if (!config) {
        drawEmpty(lastError || "reading the calibration…");
        return;
      }
      const fixed = Boolean(config.T_base_cam);
      const live = !fixed && livePose && (Date.now() - liveAt) < 3000;
      const chosen = fixed ? config.T_base_cam : (live ? predictedPose() : (lastPose || null));
      if (!chosen) {
        drawEmpty(config.marker
          ? "point the camera at the base tag"
          : "fit a fixed rig or configure the base tag");
        return;
      }
      const pose = matrixPose(chosen);
      let camera;
      try {
        camera = makeCamera(config.calibration, video.videoWidth, video.videoHeight);
      } catch (error) {
        lastError = error.message;
        drawEmpty(error.message);
        return;
      }
      const label = fixed ? "fixed rig"
        : (live ? (motionSeen ? "base tag + gyro" : "base tag")
          : (lastPoseLabel || "last tap"));
      drawBand(camera, pose, label);
    }

    async function apply() {
      if (!visible) return;
      if (loading) return;
      loading = true;
      stopDetection();
      livePose = null;
      baseCamera = null;
      try {
        await load();
        detector = null;
        lastError = "";
        draw();
        startDetection();
      } catch (error) {
        lastError = error.message;
        status("reach overlay: " + error.message, "bad");
        drawEmpty("reach overlay: " + error.message);
      } finally {
        loading = false;
      }
    }

    function toggle() {
      visible = !visible;
      canvas.classList.toggle("on", visible);
      canvas.hidden = !visible;
      if (visible) {
        // Ask for the gyro while the toggle click is still a user gesture.
        if (!config || !config.T_base_cam) requestMotionPermission();
        apply();
      } else {
        stopDetection();
        livePose = null;
        baseCamera = null;
        context.clearRect(0, 0, canvas.width, canvas.height);
      }
      return visible;
    }

    function setPose(T, label) {
      if (!Array.isArray(T) || T.length < 4 || !Array.isArray(T[2])) return;
      lastPose = T;
      lastPoseLabel = label || "last tap";
      if (visible && !(config && config.T_base_cam)) draw();
    }

    window.addEventListener("resize", () => { if (visible) draw(); });
    if (window.ResizeObserver) {
      // The log pane can grow and shrink the stage; the canvas backing store
      // and the letterboxed content box change with it.
      new ResizeObserver(() => { if (visible) draw(); }).observe(canvas);
    }
    video.addEventListener("loadedmetadata", () => {
      if (!visible) return;
      draw();
      startDetection();
    });
    video.addEventListener("resize", () => { if (visible) draw(); });
    video.addEventListener("emptied", () => {
      // The camera was stopped: do not leave the last band over a black stage.
      stopDetection();
      livePose = null;
      baseCamera = null;
      if (visible) drawEmpty("start the camera first");
    });
    document.addEventListener("visibilitychange", () => {
      if (document.visibilityState === "visible") {
        if (visible) startDetection();
      } else {
        stopDetection();
      }
    });
    // The rig panel can fit a camera after this page loaded; use it at once.
    window.addEventListener("rebot:rig", () => {
      if (!visible) return;
      stopDetection();
      detector = null;
      livePose = null;
      baseCamera = null;
      config = null;
      apply();
    });

    return {
      toggle: toggle,
      setPose: setPose,
      get visible() { return visible; },
    };
  }

  if (typeof window !== "undefined") {
    window.ReachOverlay = { create: create, computeReachBand: computeReachBand };
  }

  // Node sees the geometry without a DOM: the same code that runs in the page
  // can be checked against `rebot_ik.py` in the test harness.
  if (typeof module !== "undefined" && module.exports) {
    module.exports = {
      computeReachBand: computeReachBand,
      forward: forward,
      inverseKinematics: inverseKinematics,
      projectPoint: function (calibration, T, videoWidth, videoHeight, point) {
        return project(makeCamera(calibration, videoWidth, videoHeight), matrixPose(T), point);
      },
    };
  }
})();
