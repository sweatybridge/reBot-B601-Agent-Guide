/* Draw the arm's reachable band on top of the camera preview.
 *
 * The video already shows the table; the only missing piece is which part of
 * that table the arm can serve. This module adds a canvas over the video and
 * paints an annulus between the arm's minimum and maximum reach, projected
 * through the same pinhole camera the planner uses.
 *
 * The heavy parts are split out:
 *
 *   * `arm-model.js` holds the chain, FK/IK and the reach-band search;
 *   * `geometry.js` holds the pinhole/Brown-Conrady projection and ray math;
 *   * this module loads the config, drives the camera pose, and paints the band.
 *
 * The server only supplies numbers it already has: the calibration (K, dist)
 * and, when a fixed-camera rig is fitted, `T_base_cam`. In handheld mode the
 * browser ArUco detector solves `T_base_cam`; tag-only is the default, and
 * `?filter=ekf` opts into IMU fusion between tag frames. A page that got a pose
 * some other way can hand it over with
 * `setPose()`; no page needs to today, because `/live` is fixed-rig only and the
 * phone detects its own tag.
 *
 * Why the reach band is computed rather than guessed: the interesting shape is
 * not a circle. The arm cannot fold its wrist close to its own base while
 * keeping the tool pointing down, so the annulus has a real inner edge; joint 1
 * stops 40 deg short of a full turn, so it also has a wedge-shaped gap.
 */
(function () {
  "use strict";

  // Tag-only is the default. `?filter=ekf` opts into the separate ekf.js and
  // the IMU; `?filter=tag` remains an explicit way to say the same default.
  const POSE_FILTER = (typeof location !== "undefined" && location.search
    && new URLSearchParams(location.search).get("filter") === "ekf") ? "ekf" : "tag";

  const G = (typeof window !== "undefined" && window.ReachGeometry)
    || (typeof require === "function" ? require("./geometry.js") : null);
  const A = (typeof window !== "undefined" && window.ReachArm)
    || (typeof require === "function" ? require("./arm-model.js") : null);
  if (!G || !A) throw new Error("reach geometry/arm modules are not loaded");
  const matMul = G.matMul;
  const matVec = G.matVec;
  const axisAngle = G.axisAngle;
  const rotationFromPose = G.rotationFromPose;
  const blendOrientation = G.blendOrientation;
  const makeCamera = G.makeCamera;
  const project = G.project;
  const matrixPose = G.matrixPose;
  const computeReachBand = A.computeReachBand;
  const forward = A.forward;
  const inverseKinematics = A.inverseKinematics;
  const configStore = (typeof window !== "undefined" && window.RebotConfig) || null;

  /* ---------------------------------------------------------------- drawing */
  const COLOR = "69, 201, 138";           // the page's --good (nominal pitch)
  const COLOR_EXT = "240, 180, 41";       // the page's --warn (wider pitch fallback)

  function create(options) {
    const video = options.video;
    const canvas = options.canvas;
    const status = options.onStatus || function () {};
    const token = options.token || (() => "");
    // A page that can hand the overlay a pose but cannot detect the tag itself
    // (the workstation) says so, instead of repeating the phone's tag advice.
    const poseHint = options.poseHint || "";
    // The page owns the choice: fixed uses the rig in /reach.json, handheld
    // solves the tag. A fitted rig being present no longer forces fixed mode,
    // so the phone's Camera chip can switch without a server restart.
    const mode = options.mode || (() => "fixed");
    const isFixed = () => mode() === "fixed";
    const context = canvas.getContext("2d");
    const useEkf = POSE_FILTER !== "tag" && Boolean(window.RebotEkf);
    let ekf = null;

    let visible = false;
    let loading = false;
    let config = null;              // /reach.json
    let band = null;                // computed annulus, cached by flange height + orientation
    let bandKey = "";
    let heightM = null;             // page override of hover_m, metres above the worktop
    let toolAxis = null;            // page override of tool_axis ("current" | "x+")
    let heightTimer = 0;            // coalesces band recomputes during a height drag
    let lastPose = null;            // T_base_cam from the last plan (handheld)
    let lastPoseLabel = "";
    let lastError = "";
    let detector = null;            // browser ArUco, handheld only
    let livePose = null;            // T_base_cam from the base tag, this frame
    let liveAt = 0;
    let detectTimer = null;
    let detectFrame = 0;
    let detectMisses = 0;
    let lossLogged = false;
    let baseCamera = null;          // R_base_cam propagated by the phone gyro
    let basePosition = null;        // p_base_cam predicted from accel + tags
    let baseVelocity = [0, 0, 0];   // m/s in the base frame
    let lastTagAt = 0;
    let lastTagPosition = null;
    let motionStarted = false;
    let motionSeen = false;
    let motionPermission = "unknown";
    let motionRequest = null;
    let motionLogged = false;
    let motionStatusLogged = false;
    let lastMotionAt = 0;
    let animationId = 0;
    let drawFrame = 0;

    function effectiveHeight() {
      if (heightM !== null) return heightM;
      return Number(config && config.hover_m);
    }

    function effectiveToolAxis() {
      if (toolAxis !== null) return toolAxis;
      if (typeof options.toolAxis === "function") {
        const value = options.toolAxis();
        if (value) return String(value);
      }
      return String((config && config.tool_axis) || "current");
    }

    function recomputeBand() {
      if (!config) return false;
      const planeZ = Number(config.plane_z);
      const hoverM = effectiveHeight();
      const tipM = Number(config.tip_m);
      const axis = effectiveToolAxis();
      const key = [planeZ, hoverM, tipM, axis].join("|");
      if (bandKey !== key || !band) {
        band = computeReachBand(planeZ, hoverM, tipM, axis);
        bandKey = key;
      }
      return Boolean(band);
    }

    async function load() {
      if (!configStore) throw new Error("the reach config loader is not loaded");
      config = await configStore.load(token);
      if (!recomputeBand()) throw new Error("the arm model has no reachable band at this height");
      return config;
    }

    function clearHeightTimer() {
      if (heightTimer) {
        clearTimeout(heightTimer);
        heightTimer = 0;
      }
    }

    function applyHeight() {
      heightTimer = 0;
      if (!visible || !config) return;
      if (!recomputeBand()) {
        lastError = "the arm model has no reachable band at this height";
        drawEmpty(lastError);
        return;
      }
      lastError = "";
      draw();
    }

    function setHeight(value) {
      const number = Number(value);
      if (!Number.isFinite(number)) return;
      const next = Math.max(0, number);
      if (heightM !== null && Math.abs(heightM - next) < 1e-9) return;
      heightM = next;
      if (!visible || !config) return;
      // The band costs a radial IK scan. Coalesce a drag into one recompute per
      // ~110 ms; the final value always gets drawn because the timer is trailing.
      if (heightTimer) return;
      heightTimer = window.setTimeout(applyHeight, 110);
    }

    function setToolAxis(value) {
      const next = value ? String(value) : "current";
      if (toolAxis === next) return;
      toolAxis = next;
      if (!visible || !config) return;
      clearHeightTimer();
      applyHeight();
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

    // The stream can be a quarter turn from the calibration. `options.turn` is
    // the correction -- the same number the host measures and the tap solve uses
    // (`/reach.json`'s `stream_turn.turns`) -- so the only place this module has
    // to reverse it is when it goes the other way, from the calibration grid
    // into the frame the video is displaying.
    function frameTurn() {
      const value = typeof options.turn === "function" ? Number(options.turn()) : NaN;
      return Number.isFinite(value) ? ((Math.round(value) % 4) + 4) % 4 : 0;
    }

    function frameCamera() {
      if (frameTurn() % 2) {
        // A quarter turn: the calibrated grid is the frame's grid turned back.
        return makeCamera(config.calibration, config.calibration.image_size[0],
          config.calibration.image_size[1]);
      }
      return makeCamera(config.calibration, video.videoWidth, video.videoHeight);
    }

    function projectToFrame(camera, pose, point) {
      const pixel = project(camera, pose, point);
      if (!pixel) return null;
      const turn = frameTurn();
      if (!turn) return pixel;
      const calibration = config.calibration.image_size;
      if (turn % 2) {
        // An odd turn projects on the calibration grid, so the mapped pixel is
        // in the *turned calibration* size. The canvas is sized to the
        // delivered frame; scale that grid onto the frame, or a stream that
        // arrives at a different resolution moves the band.
        const at = G.mapPixel(pixel.u, pixel.v, calibration, (4 - turn) % 4);
        return {
          u: at[0] * (video.videoWidth / calibration[1]),
          v: at[1] * (video.videoHeight / calibration[0]),
        };
      }
      // An even turn projects on the delivered frame already; rotate in that
      // same pixel grid rather than the calibration's.
      const at = G.mapPixel(pixel.u, pixel.v, [video.videoWidth, video.videoHeight],
                            (4 - turn) % 4);
      return { u: at[0], v: at[1] };
    }

    function ringPoint(camera, pose, radius, angle, box) {
      return toCanvas(projectToFrame(camera, pose,
        [radius * Math.cos(angle), radius * Math.sin(angle), band.tipHeight]), box);
    }

    function sampleRing(camera, pose, radius, sector, samples, box) {
      const pts = [];
      for (let index = 0; index <= samples; index += 1) {
        const angle = -sector + (2 * sector * index) / samples;
        const p = ringPoint(camera, pose, radius, angle, box);
        if (p) pts.push(p);
      }
      return pts;
    }

    function fillAnnulus(outer, inner, colorRgb, alpha) {
      if (outer.length <= 1 || inner.length <= 1) return;
      context.beginPath();
      context.moveTo(outer[0].u, outer[0].v);
      for (let index = 1; index < outer.length; index += 1) {
        context.lineTo(outer[index].u, outer[index].v);
      }
      for (let index = inner.length - 1; index >= 0; index -= 1) {
        context.lineTo(inner[index].u, inner[index].v);
      }
      context.closePath();
      context.fillStyle = "rgba(" + colorRgb + ", " + alpha + ")";
      context.fill();
    }

    function strokeRing(ring, colorRgb) {
      if (ring.length < 2) return;
      context.setLineDash([7, 5]);
      context.lineWidth = 2;
      context.strokeStyle = "rgba(" + colorRgb + ", 0.95)";
      context.beginPath();
      context.moveTo(ring[0].u, ring[0].v);
      for (let index = 1; index < ring.length; index += 1) {
        context.lineTo(ring[index].u, ring[index].v);
      }
      context.stroke();
      context.setLineDash([]);
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
      const nom = band.nominal;
      const hasYellowInner = Boolean(nom && band.inner < nom.inner - 0.005);
      const hasYellowOuter = Boolean(nom && band.outer > nom.outer + 0.005);

      if (nom) {
        const nomOuter = sampleRing(camera, pose, nom.outer, sector, samples, box);
        const nomInner = sampleRing(camera, pose, nom.inner, sector, samples, box);
        if (hasYellowInner) {
          const extInner = sampleRing(camera, pose, band.inner, sector, samples, box);
          fillAnnulus(nomInner, extInner, COLOR_EXT, 0.14);
          strokeRing(extInner, COLOR_EXT);
        }
        if (hasYellowOuter) {
          const extOuter = sampleRing(camera, pose, band.outer, sector, samples, box);
          fillAnnulus(extOuter, nomOuter, COLOR_EXT, 0.14);
          strokeRing(extOuter, COLOR_EXT);
        }
        fillAnnulus(nomOuter, nomInner, COLOR, 0.15);
        strokeRing(nomOuter, COLOR);
        strokeRing(nomInner, COLOR);
      } else {
        // Only the pitched (yellow) band is reachable at this height.
        const extOuter = sampleRing(camera, pose, band.outer, sector, samples, box);
        const extInner = sampleRing(camera, pose, band.inner, sector, samples, box);
        fillAnnulus(extOuter, extInner, COLOR_EXT, 0.15);
        strokeRing(extOuter, COLOR_EXT);
        strokeRing(extInner, COLOR_EXT);
      }

      // The base, so the band has an obvious centre even when it is off-screen.
      const base = toCanvas(projectToFrame(camera, pose, [0, 0, band.tipHeight]), box);
      if (base) {
        context.beginPath();
        context.arc(base.u, base.v, 4, 0, Math.PI * 2);
        context.fillStyle = "rgba(" + (nom ? COLOR : COLOR_EXT) + ", 0.95)";
        context.fill();
      }

      // A two-line legend in the top-left of the *video*, not of the letterbox.
      // Two lines because a phone held in landscape letterboxes the preview to
      // a narrow strip, and one long line would be cut off by the clip.
      const isTopDown = band.toolAxis === "x+";
      const pitchHint = !nom
        ? "pitched " + (band.extendedPitchDeg || (isTopDown ? 60 : 30)) + "°"
        : ((hasYellowInner || hasYellowOuter)
          ? (isTopDown ? "90° / yellow 60°" : "0° / yellow ≤30°")
          : "");
      const first = "arm reach · " + band.inner.toFixed(2) + "–" + band.outer.toFixed(2) + " m"
        + (isTopDown ? " · top-down" : "");
      const second = "tip " + Math.round((band.tipHeight - Number(config.plane_z)) * 1000)
        + " mm up" + (pitchHint ? " · " + pitchHint : "") + (label ? " · " + label : "");
      context.font = "500 12.5px system-ui, -apple-system, Segoe UI, Roboto, sans-serif";
      const textWidth = Math.max(context.measureText(first).width,
        context.measureText(second).width);
      const x = box.left + 12;
      const y = box.top + 12;
      context.fillStyle = "rgba(16, 20, 26, 0.86)";
      context.strokeStyle = "rgba(" + (nom ? COLOR : COLOR_EXT) + ", 0.5)";
      context.lineWidth = 1;
      if (context.roundRect) {
        context.beginPath();
        context.roundRect(x, y, textWidth + 22, 46, 9);
        context.fill();
        context.stroke();
      } else {
        context.fillRect(x, y, textWidth + 22, 46);
      }
      context.fillStyle = nom ? "#cfe9dd" : "#f7dc9c";
      context.fillText(first, x + 11, y + 18);
      context.fillStyle = nom ? "#95c9b0" : "#e3be68";
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
      if (!useEkf) return;
      if (motionPermission !== "unknown") return;
      if (motionRequest) return;
      if (!window.DeviceMotionEvent) {
        motionPermission = "unsupported";
        noteMotion("reach overlay: no motion sensor; the band updates when the tag is seen");
        return;
      }
      // iOS gates the motion sensors behind a permission prompt that must be
      // requested from a user gesture. toggle() calls this before awaiting
      // /reach.json.
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
      if (!useEkf) return;
      if (motionStarted || !visible || !config || isFixed()) return;
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
      basePosition = null;
      baseVelocity = [0, 0, 0];
      lastTagAt = 0;
      lastTagPosition = null;
      ekf = null;
    }

    function onMotion(event) {
      if (!visible || !motionStarted || !livePose) return;

      const rate = event.rotationRate;
      let omega = null;
      if (rate) {
        const alpha = Number(rate.alpha);
        const beta = Number(rate.beta);
        const gamma = Number(rate.gamma);
        if (Number.isFinite(alpha) && Number.isFinite(beta) && Number.isFinite(gamma)) {
          // DeviceMotion: alpha is about z, beta about x, gamma about y. The
          // rear camera has x_cam=x_device, y_cam=-y_device, z_cam=-z_device.
          omega = [beta, -gamma, -alpha].map((value) => value * Math.PI / 180);
        }
      }

      const acceleration = event.acceleration;
      let accelCamera = null;
      if (acceleration) {
        const ax = Number(acceleration.x);
        const ay = Number(acceleration.y);
        const az = Number(acceleration.z);
        if (Number.isFinite(ax) && Number.isFinite(ay) && Number.isFinite(az)) {
          accelCamera = [ax, -ay, -az];
        }
      }
      if (!omega && !accelCamera) return;

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

      // onMotion is only registered under useEkf (startMotion gates on it), so
      // the EKF prediction is the only live path here. The former non-EKF IMU
      // propagation below this point was dead code and has been removed; the
      // tag-only pose tracking in applyTagPose/predictedPose is separate and
      // still used when the EKF is off.
      if (ekf) ekf.predict(omega, accelCamera, dt);
    }

    function applyTagPose(T) {
      const now = performance.now();
      const p = [T[0][3], T[1][3], T[2][3]];

      if (useEkf) {
        if (!ekf) {
          ekf = window.RebotEkf.create({
            positionNoise: 0.030,
            orientationNoise: 0.030,
            velocityNoise: 0.15,
            accelNoise: 0.60,
          });
        }
        ekf.updateTag(T, now);
        lastMotionAt = now;
        return;
      }

      if (lastTagPosition && lastTagAt && now - lastTagAt > 30) {
        const dt = (now - lastTagAt) / 1000;
        const measured = [
          (p[0] - lastTagPosition[0]) / dt,
          (p[1] - lastTagPosition[1]) / dt,
          (p[2] - lastTagPosition[2]) / dt,
        ];
        baseVelocity = [
          Math.max(-3, Math.min(3, 0.5 * baseVelocity[0] + 0.5 * measured[0])),
          Math.max(-3, Math.min(3, 0.5 * baseVelocity[1] + 0.5 * measured[1])),
          Math.max(-3, Math.min(3, 0.5 * baseVelocity[2] + 0.5 * measured[2])),
        ];
      } else {
        baseVelocity = [0, 0, 0];
      }
      basePosition = p;
      lastTagPosition = p;
      lastTagAt = now;

      const target = rotationFromPose(T);
      if (!baseCamera || !motionSeen) {
        // No motion samples: follow the tag directly, as the old overlay did.
        baseCamera = target;
      } else {
        baseCamera = blendOrientation(baseCamera, target, 0.6);
      }
      lastMotionAt = now;
    }

    function predictedPose() {
      if (!livePose) return null;
      if (useEkf) {
        const pose = ekf ? ekf.pose() : null;
        if (!pose) return livePose;
        const R = pose.rotation;
        const p = pose.position;
        return [
          [R[0][0], R[0][1], R[0][2], p[0]],
          [R[1][0], R[1][1], R[1][2], p[1]],
          [R[2][0], R[2][1], R[2][2], p[2]],
          [0, 0, 0, 1],
        ];
      }
      if (!baseCamera) return livePose;
      const R = baseCamera;
      const p = basePosition || [livePose[0][3], livePose[1][3], livePose[2][3]];
      return [
        [R[0][0], R[0][1], R[0][2], p[0]],
        [R[1][0], R[1][1], R[1][2], p[1]],
        [R[2][0], R[2][1], R[2][2], p[2]],
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
      if (!visible || !config || isFixed()) return;
      if (livePose && Date.now() - liveAt < 3000) {
        draw();
        animationId = requestAnimationFrame(animate);
      }
    }

    function startAnimation() {
      if (animationId || !visible || !config || isFixed() || !motionSeen) return;
      animationId = requestAnimationFrame(animate);
    }

    function stopDetection() {
      if (detectTimer) {
        clearTimeout(detectTimer);
        detectTimer = null;
      }
      if (detectFrame && typeof video.cancelVideoFrameCallback === "function") {
        video.cancelVideoFrameCallback(detectFrame);
      }
      detectFrame = 0;
      stopMotion();
      stopAnimation();
    }

    function startDetection() {
      if (!visible || !config || isFixed()) return;
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
      if (detectTimer || detectFrame) {
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

      function scheduleDetect() {
        if (!visible || !config || isFixed() || detectTimer || detectFrame) return;
        if (typeof video.requestVideoFrameCallback === "function") {
          // One tag solve per camera frame, not a fixed 320 ms timer: the
          // absolute anchor is then only one frame old when the phone moves.
          detectFrame = video.requestVideoFrameCallback(() => {
            detectFrame = 0;
            tick();
          });
        } else {
          detectTimer = setTimeout(tick, 66);
        }
      }

      function tick() {
        detectTimer = null;
        detectFrame = 0;
        if (!visible || !config || isFixed()) return;
        if (video.videoWidth && video.videoHeight) {
          let found = null;
          try {
            found = detector.detect();
          } catch (error) {
            lastError = error.message;
          }
          if (found && found.T_base_cam) {
            applyTagPose(found.T_base_cam);
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
        scheduleDetect();
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
      if (!band) {
        drawEmpty("the arm model has no reachable band at this height");
        return;
      }
      const fixed = isFixed();
      const live = !fixed && livePose && (Date.now() - liveAt) < 3000;
      const chosen = fixed ? config.T_base_cam : (live ? predictedPose() : (lastPose || null));
      if (!chosen) {
        drawEmpty(poseHint || (fixed
          ? "no fixed rig fitted: fit one with Rig"
          : (config.marker
            ? "point the camera at the base tag"
            : "fit a fixed rig or configure the base tag")));
        return;
      }
      const pose = matrixPose(chosen);
      let camera;
      try {
        camera = frameCamera();
      } catch (error) {
        lastError = error.message;
        drawEmpty(error.message);
        return;
      }
      const label = fixed ? "fixed rig"
        : (live ? (useEkf ? "base tag + EKF" : "base tag")
          : (lastPoseLabel || "last tap"));
      drawBand(camera, pose, label);
    }

    async function apply() {
      if (!visible) return;
      if (loading) return;
      loading = true;
      stopDetection();
      clearHeightTimer();
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
        // Ask for the motion sensors while the toggle click is still a gesture.
        if (!config || !isFixed()) requestMotionPermission();
        apply();
      } else {
        stopDetection();
        clearHeightTimer();
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
      if (visible && !isFixed()) draw();
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
      if (configStore) configStore.invalidate();
      if (!visible) return;
      stopDetection();
      clearHeightTimer();
      detector = null;
      livePose = null;
      baseCamera = null;
      config = null;
      band = null;
      bandKey = "";
      apply();
    });

    function projectPoint(point) {
      if (!config || !Array.isArray(point) || point.length < 3) return null;
      if (!video.videoWidth || !video.videoHeight) return null;
      const fixed = isFixed();
      const live = !fixed && livePose && (Date.now() - liveAt) < 3000;
      const chosen = fixed ? config.T_base_cam : (live ? predictedPose() : null);
      if (!chosen) return null;
      try {
        const camera = frameCamera();
        const pixel = projectToFrame(camera, matrixPose(chosen), point);
        return pixel ? [pixel.u, pixel.v] : null;
      } catch (error) {
        return null;
      }
    }

    async function projectPointLoaded(point) {
      // The agent's arm_to_pixel must work without the overlay toggled on:
      // projectPoint reads the private `config`, which apply() only fills
      // while visible. Load it on demand (no band recompute -- projection
      // needs the rig, not a reachable band at the current height).
      if (!config) {
        if (!configStore) throw new Error("the reach config loader is not loaded");
        config = await configStore.load(token);
      }
      return projectPoint(point);
    }

    return {
      toggle: toggle,
      // The page's camera mode changed: restart detection/pose selection.
      refresh: apply,
      setPose: setPose,
      projectPoint: projectPoint,
      projectPointLoaded: projectPointLoaded,
      setHeight: setHeight,
      setToolAxis: setToolAxis,
      get height() { return heightM; },
      get toolAxis() { return effectiveToolAxis(); },
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
