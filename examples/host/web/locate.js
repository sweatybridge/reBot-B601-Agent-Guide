/* Locate a tap in the arm frame, in the page.
 *
 * The phone already has the camera pose: a fitted rig from `/reach.json`, or the
 * base tag solved in `aruco.js`. This module turns that plus the calibration and
 * the worktop plane into the one thing the planner needs: the tool tip position
 * in the arm base frame. The capture server only enriches and validates it; no
 * frame has to make a round trip.
 */
(function () {
  "use strict";

  function makeCamera(calibration, width, height) {
    const source = calibration.image_size;
    if (!source || source.length !== 2) {
      throw new Error("the calibration has no image_size");
    }
    const sx = width / source[0];
    const sy = height / source[1];
    if (Math.abs(sx - sy) / Math.max(sx, sy) > 0.01) {
      throw new Error("calibration is for " + source[0] + "x" + source[1]
        + " but the camera is " + width + "x" + height);
    }
    const K = calibration.K;
    const dist = calibration.dist || [];
    return {
      fx: K[0][0] * sx, fy: K[1][1] * sy,
      cx: K[0][2] * sx, cy: K[1][2] * sy,
      k1: dist[0] || 0, k2: dist[1] || 0,
      p1: dist[2] || 0, p2: dist[3] || 0, k3: dist[4] || 0,
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

  function tipTarget(calibration, T, u, v, planeZ, tapHeight, videoWidth, videoHeight) {
    const width = videoWidth || calibration.image_size[0];
    const heightPx = videoHeight || calibration.image_size[1];
    const camera = makeCamera(calibration, width, heightPx);
    const height = Number(tapHeight);
    if (!Number.isFinite(height)) throw new Error("the tap height is not a number");
    // Direct intersection with the raised plane: the tip is the point under the
    // tapped pixel at that height, so its x/y move along the ray as the height
    // changes. This is not the same as lifting the worktop point vertically.
    const tip = rayPlane(camera, T, u, v, Number(planeZ) + height).point;
    return { tip: tip, pixel: [u, v], tap: { u: u, v: v } };
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

  function solveTag(pixels, width, height, calibration, marker, planeZ) {
    if (!window.ArucoPose) throw new Error("the browser ArUco detector is not loaded");
    const found = window.ArucoPose.detectFrame(pixels, width, height);
    if (!found || !found.corners) {
      throw new Error("the base tag is not in the tapped frame");
    }
    const scaled = scaleCalibration(calibration, width, height);
    const pose = window.ArucoPose.poseFromCorners(found.corners, scaled, marker, planeZ);
    if (!pose || !pose.T_base_cam) {
      throw new Error("the base tag pose could not be solved");
    }
    return pose.T_base_cam;
  }

  function grabFrame(video) {
    const width = video.videoWidth;
    const height = video.videoHeight;
    if (!width || !height) throw new Error("the camera has no frame yet");
    const canvas = document.createElement("canvas");
    canvas.width = width;
    canvas.height = height;
    const context = canvas.getContext("2d");
    context.drawImage(video, 0, 0, width, height);
    return { data: context.getImageData(0, 0, width, height).data, width: width, height: height };
  }

  function create(options) {
    const video = options.video;
    const token = options.token || (() => "");
    const mode = options.mode || (() => "handheld");
    const status = options.onStatus || function () {};
    let config = null;
    let loading = null;

    function headers() {
      const value = typeof token === "function" ? token() : token;
      return value ? { "X-Rebot-Token": value } : {};
    }

    async function load() {
      if (config) return config;
      if (loading) return loading;
      loading = (async () => {
        const response = await fetch("/reach.json", {
          headers: headers(), cache: "no-store",
        });
        const data = await response.json().catch(() => ({}));
        if (!response.ok || !data.ok) {
          throw new Error(data.error || ("the host answered " + response.status));
        }
        config = data;
        return data;
      })();
      try {
        return await loading;
      } finally {
        loading = null;
      }
    }

    async function locate(tap, tapHeight) {
      const data = await load();
      let T = null;
      if (mode() === "fixed") {
        T = data.T_base_cam;
        if (!T) throw new Error("fixed camera mode needs a fitted rig");
      } else {
        if (!data.marker) {
          throw new Error("handheld mode needs the base tag; fit a rig or switch camera mode");
        }
        status("locating the base tag in this frame", "");
        const frame = grabFrame(video);
        T = solveTag(frame.data, frame.width, frame.height,
          data.calibration, data.marker, data.plane_z);
      }
      return tipTarget(data.calibration, T, tap.u, tap.v, data.plane_z, tapHeight,
        video.videoWidth, video.videoHeight);
    }

    // A rig fitted after this page loaded must be picked up by the next tap.
    window.addEventListener("rebot:rig", () => { config = null; });

    return { locate: locate, get config() { return config; } };
  }

  if (typeof window !== "undefined") {
    window.BrowserLocate = {
      create: create,
      makeCamera: makeCamera,
      rayPlane: rayPlane,
      tipTarget: tipTarget,
      scaleCalibration: scaleCalibration,
    };
  }

  if (typeof module !== "undefined" && module.exports) {
    module.exports = {
      makeCamera: makeCamera,
      undistort: undistort,
      rayPlane: rayPlane,
      tipTarget: tipTarget,
      scaleCalibration: scaleCalibration,
    };
  }
})();
