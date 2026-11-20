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
      : G.shapeTurn(calibration.image_size, [width, heightPx]);
    if (!(applied >= 0)) {
      // Neither the host nor the frame shape knows the turn: assuming 0 here
      // is a silent wrong target on a turned stream.
      throw new Error("the stream turn is unknown and the frame shape does not determine it");
    }
    let at;
    let camera;
    if (applied) {
      // Rotate the tap onto the upright video grid, then scale that grid onto
      // the calibration grid: a turned stream can also be resized, and K stays
      // on the grid it was fitted for. Rotating alone leaves the tap in video
      // pixels while K expects calibration pixels.
      const rotated = G.mapPixel(u, v, [width, heightPx], applied);
      const uprightW = applied % 2 ? heightPx : width;
      const uprightH = applied % 2 ? width : heightPx;
      at = [rotated[0] * calibration.image_size[0] / uprightW,
            rotated[1] * calibration.image_size[1] / uprightH];
      camera = makeCamera(calibration, calibration.image_size[0], calibration.image_size[1]);
    } else {
      at = [u, v];
      camera = makeCamera(calibration, width, heightPx);
    }

    // Direct intersection with the raised plane: the tip is the point under the
    // tapped pixel at that height, so its x/y move along the ray as the height
    // changes. This is not the same as lifting the worktop point vertically.
    const tip = rayPlane(camera, T, at[0], at[1], Number(planeZ) + height).point;
    return { tip: tip, pixel: [u, v], tap: { u: u, v: v }, turn: applied };
  }

  function rotateFrame(pixels, width, height, turns) {
    // Rotate the frame onto the calibration grid, clockwise, matching
    // mapPixel and the host's upright_frame (turn 1 = 90 deg CW). A handheld
    // phone delivers a quarter-turned frame; detecting on it against an
    // upright K is what made the handheld tag solve refuse turned streams.
    const steps = ((Math.round(turns) % 4) + 4) % 4;
    if (!steps) return { data: pixels, width: width, height: height };
    let src = document.createElement("canvas");
    src.width = width;
    src.height = height;
    src.getContext("2d").putImageData(new ImageData(pixels, width, height), 0, 0);
    let w = width;
    let h = height;
    for (let i = 0; i < steps; i += 1) {
      const dst = document.createElement("canvas");
      dst.width = h;
      dst.height = w;
      const ctx = dst.getContext("2d");
      ctx.translate(h, 0);
      ctx.rotate(Math.PI / 2);
      ctx.drawImage(src, 0, 0);
      src = dst;
      const swap = w;
      w = h;
      h = swap;
    }
    return { data: src.getContext("2d").getImageData(0, 0, w, h).data, width: w, height: h };
  }

  function solveTag(pixels, width, height, calibration, marker, planeZ, turn) {
    if (!window.ArucoPose) throw new Error("the browser ArUco detector is not loaded");
    let applied = Number.isFinite(Number(turn))
      ? ((Math.round(Number(turn)) % 4) + 4) % 4
      : G.shapeTurn(calibration.image_size, [width, height]);
    if (!(applied >= 0)) applied = 0;
    const frame = applied ? rotateFrame(pixels, width, height, applied)
      : { data: pixels, width: width, height: height };
    const found = window.ArucoPose.detectFrame(frame.data, frame.width, frame.height);
    if (!found || !found.corners) {
      throw new Error("the base tag is not in the tapped frame");
    }
    const scaled = scaleCalibration(calibration, frame.width, frame.height);
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
        // Prefer the host-measured turn (its sign is resolved from the tag);
        // fall back to the page's turn() and then the frame shape.
        const measured = data.stream_turn && Number.isFinite(Number(data.stream_turn.turns))
          ? ((Math.round(Number(data.stream_turn.turns)) % 4) + 4) % 4
          : turn();
        T = solveTag(frame.data, frame.width, frame.height,
          data.calibration, data.marker, data.plane_z, measured);
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
      solveTag: solveTag,
      rotateFrame: rotateFrame,
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

