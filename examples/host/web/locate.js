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

  const G = (typeof window !== "undefined" && window.ReachGeometry)
    || (typeof require === "function" ? require("./geometry.js") : null);
  if (!G) throw new Error("reach geometry is not loaded");
  const makeCamera = G.makeCamera;
  const undistort = G.undistort;
  const rayPlane = G.rayPlane;
  const scaleCalibration = G.scaleCalibration;
  const configStore = (typeof window !== "undefined" && window.RebotConfig) || null;
  const videoTools = (typeof window !== "undefined" && window.RebotVideo) || null;

  // `turn` is the correction (see geometry.js's shapeTurn / the host's
  // /reach.json `stream_turn`): the quarter turns to apply to the delivered
  // frame to reach the calibration grid. The tap is moved by the same number.
  function tipTarget(calibration, T, u, v, planeZ, tapHeight, videoWidth, videoHeight, turn) {
    const width = videoWidth || calibration.image_size[0];
    const heightPx = videoHeight || calibration.image_size[1];
    const height = Number(tapHeight);
    if (!Number.isFinite(height)) throw new Error("the tap height is not a number");

    const applied = Number.isFinite(Number(turn))
      ? ((Math.round(Number(turn)) % 4) + 4) % 4
      : Math.max(0, G.shapeTurn(calibration.image_size, [width, heightPx]));
    const source = applied ? calibration.image_size : [width, heightPx];
    const at = applied ? G.mapPixel(u, v, [width, heightPx], applied) : [u, v];
    const camera = makeCamera(calibration, source[0], source[1]);

    // Direct intersection with the raised plane: the tip is the point under the
    // tapped pixel at that height, so its x/y move along the ray as the height
    // changes. This is not the same as lifting the worktop point vertically.
    const tip = rayPlane(camera, T, at[0], at[1], Number(planeZ) + height).point;
    return { tip: tip, pixel: [u, v], tap: { u: u, v: v }, turn: applied };
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

  function create(options) {
    const video = options.video;
    const token = options.token || (() => "");
    const mode = options.mode || (() => "handheld");
    // The correction this page must apply: /reach.json's `stream_turn.turns`.
    // Left out, the frame shape decides.
    const turn = options.turn || (() => undefined);
    const status = options.onStatus || function () {};

    async function locate(tap, tapHeight) {
      if (!configStore) throw new Error("the reach config loader is not loaded");
      const data = await configStore.load(token);
      let T = null;
      if (mode() === "fixed") {
        T = data.T_base_cam;
        if (!T) throw new Error("fixed camera mode needs a fitted rig");
      } else {
        if (!data.marker) {
          throw new Error("handheld mode needs the base tag; fit a rig or switch camera mode");
        }
        status("locating the base tag in this frame", "");
        if (!videoTools) throw new Error("the video helpers are not loaded");
        const frame = videoTools.grabImageData(video);
        T = solveTag(frame.data, frame.width, frame.height,
          data.calibration, data.marker, data.plane_z);
      }
      return tipTarget(data.calibration, T, tap.u, tap.v, data.plane_z, tapHeight,
        video.videoWidth, video.videoHeight, turn());
    }

    return { locate: locate, get config() { return configStore ? configStore.get() : null; } };
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

