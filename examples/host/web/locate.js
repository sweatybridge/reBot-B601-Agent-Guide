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

  function tipTarget(calibration, T, u, v, planeZ, tapHeight, videoWidth, videoHeight, turns) {
    const width = videoWidth || calibration.image_size[0];
    const heightPx = videoHeight || calibration.image_size[1];
    const height = Number(tapHeight);
    if (!Number.isFinite(height)) throw new Error("the tap height is not a number");

    // A frame can be a quarter turn from the calibration. `turns` is that turn
    // when the page knows it (the phone publishes its screen angle); otherwise
    // the shape gives an odd/even answer, and clockwise is assumed.
    const rotation = Number.isFinite(Number(turns))
      ? ((Math.round(Number(turns)) % 4) + 4) % 4
      : Math.max(0, G.turnsForShapes(calibration.image_size, [width, heightPx]));
    const source = rotation ? calibration.image_size : [width, heightPx];
    const at = rotation ? G.mapPixel(u, v, [width, heightPx], 4 - rotation) : [u, v];
    const camera = makeCamera(calibration, source[0], source[1]);

    // Direct intersection with the raised plane: the tip is the point under the
    // tapped pixel at that height, so its x/y move along the ray as the height
    // changes. This is not the same as lifting the worktop point vertically.
    const tip = rayPlane(camera, T, at[0], at[1], Number(planeZ) + height).point;
    return { tip: tip, pixel: [u, v], tap: { u: u, v: v }, rotation: rotation };
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
    // The page knows the turn (the phone publishes its screen angle); a page
    // that does not leaves this out and the shape decides.
    const turns = options.turns || (() => undefined);
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
        video.videoWidth, video.videoHeight, turns());
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
