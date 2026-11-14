/* Arm kinematics and the reach band for the phone overlay.
 *
 * The chain is transcribed from `rebot_ik.py`; everything here is DOM-free so
 * it can be checked against the Python model in Node.
 */
(function () {
  "use strict";

  const G = (typeof window !== "undefined" && window.ReachGeometry)
    || (typeof require === "function" ? require("./geometry.js") : null);
  if (!G) throw new Error("reach geometry is not loaded");
  const identity = G.identity;
  const matMul = G.matMul;
  const matVec = G.matVec;
  const rpyToMatrix = G.rpyToMatrix;
  const axisAngle = G.axisAngle;
  const rotationError = G.rotationError;

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


  if (typeof window !== "undefined") {
    window.ReachArm = {
      computeReachBand: computeReachBand,
      forward: forward,
      inverseKinematics: inverseKinematics,
      MAX_REACH: MAX_REACH,
    };
  }
  if (typeof module !== "undefined" && module.exports) {
    module.exports = {
      computeReachBand: computeReachBand,
      forward: forward,
      inverseKinematics: inverseKinematics,
      MAX_REACH: MAX_REACH,
    };
  }
})();
