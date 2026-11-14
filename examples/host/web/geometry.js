/* Shared camera and matrix geometry for the phone page.
 *
 * Used by the reach overlay and by the browser tap locator, so calibration,
 * projection and ray/plane math have one implementation.
 */
(function () {
  "use strict";

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

  /* ------------------------------------------------------------ projection */
  function makeCamera(calibration, videoWidth, videoHeight) {
    const source = calibration.image_size;
    if (!source || source.length !== 2) {
      throw new Error("the calibration has no image_size");
    }
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

  function scaleCalibration(calibration, width, height) {
    const source = calibration.image_size;
    if (!source || source.length !== 2) {
      throw new Error("the calibration has no image_size");
    }
    const sx = width / source[0];
    const sy = height / source[1];
    if (Math.abs(sx - sy) / Math.max(sx, sy) > 0.01) {
      throw new Error("calibration is for " + source[0] + "x" + source[1]
        + " but the frame is " + width + "x" + height);
    }
    if (Math.abs(sx - 1) < 1e-9 && Math.abs(sy - 1) < 1e-9) return calibration;
    const K = calibration.K;
    return {
      K: [
        [K[0][0] * sx, K[0][1] * sx, K[0][2] * sx],
        [K[1][0] * sy, K[1][1] * sy, K[1][2] * sy],
        [K[2][0], K[2][1], K[2][2]],
      ],
      dist: calibration.dist || [],
      image_size: [width, height],
    };
  }

  function undistort(u, v, camera) {
    const xd = (u - camera.cx) / camera.fx;
    const yd = (v - camera.cy) / camera.fy;
    let x = xd;
    let y = yd;
    for (let iteration = 0; iteration < 6; iteration += 1) {
      const r2 = x * x + y * y;
      const radial = 1 + camera.k1 * r2 + camera.k2 * r2 * r2
        + camera.k3 * r2 * r2 * r2;
      const dx = 2 * camera.p1 * x * y + camera.p2 * (r2 + 2 * x * x);
      const dy = camera.p1 * (r2 + 2 * y * y) + 2 * camera.p2 * x * y;
      x = (xd - dx) / radial;
      y = (yd - dy) / radial;
    }
    return [x, y];
  }

  function rayPlane(camera, T, u, v, planeZ) {
    const [x, y] = undistort(u, v, camera);
    const directionCam = [x, y, 1.0];
    const origin = [T[0][3], T[1][3], T[2][3]];
    let direction = [
      T[0][0] * directionCam[0] + T[0][1] * directionCam[1] + T[0][2] * directionCam[2],
      T[1][0] * directionCam[0] + T[1][1] * directionCam[1] + T[1][2] * directionCam[2],
      T[2][0] * directionCam[0] + T[2][1] * directionCam[1] + T[2][2] * directionCam[2],
    ];
    const norm = Math.hypot(direction[0], direction[1], direction[2]);
    if (!(norm > 1e-12)) throw new Error("degenerate tap ray");
    direction = direction.map((value) => value / norm);
    if (Math.abs(direction[2]) < 1e-6) {
      throw new Error("the tap ray is parallel to the worktop");
    }
    const distance = (Number(planeZ) - origin[2]) / direction[2];
    if (!(distance > 0)) {
      throw new Error("the tap ray points away from the worktop");
    }
    return {
      point: [
        origin[0] + distance * direction[0],
        origin[1] + distance * direction[1],
        origin[2] + distance * direction[2],
      ],
      distance: distance,
      grazing: Math.asin(Math.min(1, Math.abs(direction[2]))) * 180 / Math.PI,
    };
  }

  if (typeof window !== "undefined") {
    window.ReachGeometry = {
      identity: identity, matMul: matMul, matVec: matVec, transpose: transpose,
      rpyToMatrix: rpyToMatrix, axisAngle: axisAngle,
      rotationError: rotationError, rotationFromPose: rotationFromPose,
      blendOrientation: blendOrientation,
      makeCamera: makeCamera, project: project, matrixPose: matrixPose,
      scaleCalibration: scaleCalibration, undistort: undistort, rayPlane: rayPlane,
    };
  }
  if (typeof module !== "undefined" && module.exports) {
    module.exports = {
      identity: identity, matMul: matMul, matVec: matVec, transpose: transpose,
      rpyToMatrix: rpyToMatrix, axisAngle: axisAngle,
      rotationError: rotationError, rotationFromPose: rotationFromPose,
      blendOrientation: blendOrientation,
      makeCamera: makeCamera, project: project, matrixPose: matrixPose,
      scaleCalibration: scaleCalibration, undistort: undistort, rayPlane: rayPlane,
    };
  }
})();
