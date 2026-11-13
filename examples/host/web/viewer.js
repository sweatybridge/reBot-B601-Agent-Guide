/* Tap a point in a video and plan the arm there.
 *
 * Shared by the phone page and the workstation viewer: both decode the same
 * stream, so both can grab the frame under the finger and post it. The module
 * owns the tap marker, the "planning" state and the result panel; the page only
 * supplies the <video> element, a stage to listen on, and the token.
 *
 *   const planner = TapPlanner.create({video, stage, token, onStatus});
 *   planner.enable();
 *
 * Flow: tap -> canvas grab -> POST /capture -> POST /plan -> panel.
 * The panel's Execute button is a two-step confirm and only appears when the
 * server says execution is enabled (REBOT_EXECUTE=1).
 */
(function () {
  "use strict";

  const STYLE = `
  .tp-marker{position:fixed;width:40px;height:40px;margin:-20px 0 0 -20px;border:2px solid #f0b429;
    border-radius:50%;pointer-events:none;z-index:40;box-shadow:0 0 0 1px rgba(0,0,0,.55);}
  .tp-marker::before,.tp-marker::after{content:"";position:absolute;background:#f0b429;}
  .tp-marker::before{left:50%;top:-7px;width:2px;height:54px;margin-left:-1px;}
  .tp-marker::after{top:50%;left:-7px;height:2px;width:54px;margin-top:-1px;}
  .tp-panel{position:absolute;left:10px;right:10px;bottom:10px;z-index:45;
    background:rgba(16,20,26,.94);border:1px solid #2b323c;border-radius:12px;
    padding:10px 12px;color:#e8eaed;font:13px/1.45 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;
    box-shadow:0 8px 24px rgba(0,0,0,.45);}
  .tp-panel[hidden]{display:none;}
  .tp-panel .tp-title{font-weight:650;margin-bottom:2px;}
  .tp-panel .tp-meta{color:#9aa4b2;font-size:12px;}
  .tp-panel .tp-error{color:#ff6b6b;}
  .tp-panel .tp-row{display:flex;gap:8px;margin-top:8px;flex-wrap:wrap;}
  .tp-panel button{font:inherit;font-size:12.5px;font-weight:600;padding:8px 12px;border-radius:9px;
    border:1px solid #2b323c;background:#1b2027;color:#e8eaed;cursor:pointer;touch-action:manipulation;}
  .tp-panel button.primary{background:#4ea1ff;border-color:#4ea1ff;color:#04121f;}
  .tp-panel button.armed{background:#ff6b6b;border-color:#ff6b6b;color:#210404;}
  .tp-panel button:disabled{opacity:.45;cursor:default;}
  .tp-panel a{color:#4ea1ff;text-decoration:none;}
  .rp-open{position:fixed;right:14px;bottom:14px;z-index:44;font:600 12.5px/1 system-ui,-apple-system,
    "Segoe UI",Roboto,sans-serif;padding:10px 13px;border-radius:10px;border:1px solid #2b323c;
    background:rgba(16,20,26,.9);color:#cfd6e0;cursor:pointer;backdrop-filter:blur(6px);
    touch-action:manipulation;}
  .rp-open:hover{color:#fff;border-color:#4ea1ff;}
  .rp-overlay{position:fixed;left:0;top:0;right:0;bottom:0;z-index:60;background:rgba(6,9,13,.72);
    overflow:auto;padding:16px;}
  .rp-overlay[hidden]{display:none;}
  .rp-card{box-sizing:border-box;width:auto;max-width:560px;margin:6vh auto 24px;white-space:normal;
    background:#12171d;border:1px solid #2b323c;border-radius:14px;padding:16px 18px;color:#e8eaed;
    font:13px/1.5 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;
    box-shadow:0 20px 60px rgba(0,0,0,.6);}
  .rp-card h2{font-size:15px;margin:0 0 2px;}
  .rp-card .rp-sub{color:#9aa4b2;font-size:12px;}
  .rp-card .rp-kv{display:grid;grid-template-columns:104px 1fr;gap:3px 10px;font-size:12.5px;margin:10px 0;}
  .rp-card .rp-kv span:nth-child(odd){color:#9aa4b2;}
  .rp-card input{box-sizing:border-box;width:64px;font:inherit;font-size:12.5px;padding:5px 6px;
    border-radius:7px;border:1px solid #2b323c;background:#0d1116;color:#e8eaed;}
  .rp-card .rp-row{display:flex;gap:8px;margin-top:10px;flex-wrap:wrap;align-items:center;}
  .rp-card button{box-sizing:border-box;font:inherit;font-size:12.5px;font-weight:600;padding:8px 12px;
    border-radius:9px;border:1px solid #2b323c;background:#1b2027;color:#e8eaed;cursor:pointer;
    touch-action:manipulation;}
  .rp-card button.primary{background:#4ea1ff;border-color:#4ea1ff;color:#04121f;}
  .rp-card button.armed{background:#ff6b6b;border-color:#ff6b6b;color:#210404;}
  .rp-card button:disabled{opacity:.45;cursor:default;}
  .rp-card .rp-bar{height:6px;border-radius:3px;background:#1b2027;overflow:hidden;margin:10px 0 6px;}
  .rp-card .rp-bar i{display:block;height:100%;background:#4ea1ff;width:0;transition:width .3s ease;}
  .rp-card .rp-good{color:#45c98a;}
  .rp-card .rp-bad{color:#ff6b6b;}
  .rp-card .rp-warn{color:#f0b429;}
  .rp-card ul{margin:6px 0 0 18px;padding:0;}
  .rp-card table{width:100%;border-collapse:collapse;font-size:12px;margin-top:8px;}
  .rp-card th,.rp-card td{text-align:left;padding:3px 6px;border-bottom:1px solid #1f252d;}
  .rp-card th{color:#9aa4b2;font-weight:600;}
  .rp-card code{background:#0d1116;padding:1px 5px;border-radius:4px;font-size:11.5px;
    display:inline-block;word-break:break-all;}
  .rp-sep{height:1px;background:#1f252d;margin:14px 0 10px;}
  .rp-shot-wrap{position:relative;margin-top:8px;border-radius:8px;overflow:hidden;
    background:#0d1116;}
  .rp-shot{display:block;width:100%;height:auto;cursor:crosshair;}
  .rp-pin{position:absolute;width:20px;height:20px;margin:-10px 0 0 -10px;border:2px solid #f0b429;
    border-radius:50%;pointer-events:none;box-shadow:0 0 0 1px rgba(0,0,0,.6);}
  .rp-pin::before,.rp-pin::after{content:"";position:absolute;background:#f0b429;}
  .rp-pin::before{left:50%;top:-6px;width:2px;height:32px;margin-left:-1px;}
  .rp-pin::after{top:50%;left:-6px;height:2px;width:32px;margin-top:-1px;}
  `;

  function styleOnce() {
    if (document.getElementById("tp-style")) return;
    const style = document.createElement("style");
    style.id = "tp-style";
    style.textContent = STYLE;
    document.head.appendChild(style);
  }

  function videoPoint(video, clientX, clientY) {
    const rect = video.getBoundingClientRect();
    const width = video.videoWidth;
    const height = video.videoHeight;
    if (!width || !height) return null;
    const scale = Math.min(rect.width / width, rect.height / height);
    const offsetX = rect.left + (rect.width - width * scale) / 2;
    const offsetY = rect.top + (rect.height - height * scale) / 2;
    const u = (clientX - offsetX) / scale;
    const v = (clientY - offsetY) / scale;
    if (u < 0 || v < 0 || u > width || v > height) return null;
    return { u: Math.round(u * 100) / 100, v: Math.round(v * 100) / 100 };
  }

  /* --------------------------------------------------------------- the rig
   *
   * Fitting the camera is a two-minute machine operation with nobody at the
   * panel, so it gets its own overlay rather than a chip: one button, a state
   * card, one confirmed POST, and a progress line that keeps up with the arm.
   * The panel is created by `create()`, so both pages have it without knowing.
   */

  // How a tap becomes a point: "fixed" projects through the rig a sweep fitted;
  // "handheld" solves the camera from the base marker in the tapped frame. Both
  // end with the tool tip 10 cm above the tapped place. The page owns the choice
  // (a chip), because it applies to every tap, not to the rig card.
  let cameraMode = "handheld";

  function setMode(value) {
    cameraMode = value === "fixed" ? "fixed" : "handheld";
    return cameraMode;
  }

  function getMode() {
    return cameraMode;
  }

  function createRig(getToken) {
    styleOnce();

    function headers() {
      const value = typeof getToken === "function" ? getToken() : getToken;
      return value ? { "X-Rebot-Token": value } : {};
    }

    const open = document.createElement("button");
    open.type = "button";
    open.className = "rp-open";
    open.textContent = "Rig";
    open.title = "Fit the fixed camera from the arm";
    open.addEventListener("click", launch);
    document.body.appendChild(open);

    const overlay = document.createElement("div");
    overlay.className = "rp-overlay";
    overlay.hidden = true;
    document.body.appendChild(overlay);

    let status = null;
    let progress = null;
    let result = null;
    let running = false;
    let posting = false;
    let armedUntil = 0;
    let startedAt = 0;
    let poll = null;
    let shot = null;          // the still waiting for a tap
    let armState = null;      // {ok, q} or {ok: false, error}: can the gateway see the arm?
    let pending = false;      // a short request is in flight

    function elapsed() {
      if (!startedAt) return "";
      const seconds = Math.max(0, Math.round((Date.now() - startedAt) / 1000));
      return seconds >= 60 ? Math.floor(seconds / 60) + " min " + (seconds % 60) + " s"
        : seconds + " s";
    }

    async function load() {
      try {
        const response = await fetch("/calibrate", { headers: headers(), cache: "no-store" });
        if (!response.ok) return { ok: false, error: "status " + response.status };
        return await response.json();
      } catch (error) {
        return { ok: false, error: error.message };
      }
    }

    async function postJson(url, body) {
      try {
        const response = await fetch(url, {
          method: "POST",
          headers: Object.assign({ "Content-Type": "application/json" }, headers()),
          body: JSON.stringify(body || {}),
        });
        // Not every answer is JSON: an older server, a proxy or a plain-text
        // HTTPException. Show what it said instead of only the status code.
        const text = await response.text();
        let data;
        try {
          data = JSON.parse(text);
        } catch (error) {
          const said = (text || "").trim().slice(0, 300);
          data = { ok: false, error: said || ("server said " + response.status) };
        }
        return { ok: response.ok && data.ok !== false, data: data };
      } catch (error) {
        return { ok: false, data: { ok: false, error: error.message } };
      }
    }

    async function checkArm() {
      try {
        const response = await fetch("/pose", { headers: headers(), cache: "no-store" });
        const data = await response.json().catch(() => ({}));
        if (!response.ok || !data.ok) {
          armState = {
            ok: false,
            // "Refused" and "unreachable" are different problems: a 404 is a
            // route that is not there, a network error is a host that is not.
            refused: !data.unreachable,
            error: data.error || ("the arm gateway answered " + response.status),
          };
        } else {
          armState = { ok: true, q: (data.pose || {}).q || [] };
        }
      } catch (error) {
        armState = { ok: false, refused: false, error: error.message };
      }
      return armState;
    }

    async function capturePose() {
      if (pending) return;
      pending = true;
      render();
      const answer = await postJson("/capture/pose", {});
      pending = false;
      if (!answer.ok) {
        status = Object.assign({}, status, { error: answer.data.error || "capture failed" });
        render();
        return;
      }
      shot = { id: answer.data.id, url: answer.data.image_url,
               width: answer.data.image_size[0], height: answer.data.image_size[1],
               point: null };
      status = Object.assign({}, status, { error: null });
      render();
    }

    function escapeHtml(text) {
      return String(text == null ? "" : text).replace(/[&<>"']/g, (character) => ({
        "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;",
      }[character]));
    }

    function progressText() {
      if (!progress) return "starting…";
      const phase = progress.phase === "recording" ? "recording the sweep"
        : progress.phase === "extracting" ? "picking the frames"
        : progress.phase === "moving" ? "moving the arm"
        : progress.phase === "fitting" ? "fitting the rig"
        : "starting";
      const where = progress.pose > 0 && progress.of
        ? "pose " + progress.pose + " of " + progress.of + " · " : "";
      return escapeHtml(where + phase) + " · " + elapsed();
    }

    function resultHtml(data) {
      if (!data || data.ok === undefined) return "";
      if (data.ok) {
        const verification = data.verification || {};
        const predicted = verification.predicted_mm || {};
        const lines = ['<div class="rp-good" style="font-weight:650">Rig written</div>', '<div class="rp-kv">'];
        lines.push("<span>Fingerprint</span><span>" + escapeHtml((data.rig || {}).fingerprint || "?") + "</span>");
        lines.push("<span>Camera</span><span>" + escapeHtml(data.camera || "?") + "</span>");
        lines.push("<span>Tip</span><span>[" + (data.tip_mm || []).join(", ") + "] mm in gripper_end</span>");
        lines.push("<span>Fit</span><span>" + Number(verification.rms_px || 0).toFixed(2) + " px rms"
          + (verification.worst_px == null ? "" : " · " + Number(verification.worst_px).toFixed(1)
            + " px worst held-out") + "</span>");
        if (predicted.centre_mm != null) {
          lines.push("<span>Predicted</span><span>" + predicted.centre_mm + " mm at the centre · "
            + predicted.corner_mm + " mm at the corner</span>");
        }
        lines.push("</div>");
        lines.push('<div class="rp-sub">Taps now use this rig; nothing needs to be in the frame '
          + "but the target.</div>");
        return lines.join("");
      }

      const lines = ['<div class="rp-bad" style="font-weight:650">Not written</div>'];
      if (data.error) {
        lines.push('<div class="rp-bad" style="margin-top:2px">' + escapeHtml(data.error) + "</div>");
      }
      const verification = data.verification || {};
      if (verification.reasons && verification.reasons.length) {
        lines.push("<ul>" + verification.reasons.map((reason) => "<li>" + escapeHtml(reason) + "</li>").join("") + "</ul>");
      }
      lines.push(detectionTable(data));
      if (verification.reasons && verification.reasons.length) {
        lines.push('<div class="rp-sub" style="margin-top:8px">Nothing was written. Re-run the '
          + "sweep with the arm still and the tool tip clearly in view. If it keeps refusing, "
          + "that is the tip detector's limit, not the fit's.</div>");
      }
      return lines.join("");
    }

    function detectionTable(data) {
      const detections = data.detections || [];
      if (!detections.length) return "";
      const held = (data.verification || {}).leave_one_out_px || [];
      const rows = ["<table><tr><th>pose</th><th>tip (px)</th><th>from model</th><th>held-out</th></tr>"];
      detections.forEach((item, index) => {
        const value = held[index];
        const flag = value != null && value > 3 ? ' class="rp-bad"' : "";
        rows.push("<tr><td>" + item.pose + "</td><td>" + Number(item.u).toFixed(0) + ", "
          + Number(item.v).toFixed(0) + "</td><td>" + Number(item.offset_px || 0).toFixed(0)
          + " px</td><td" + flag + ">" + (value == null ? "—" : Number(value).toFixed(1) + " px")
          + "</td></tr>");
      });
      rows.push("</table>");
      return rows.join("");
    }

    function render() {
      const info = status || {};
      const rig = info.rig;
      const rows = ['<h2>Camera rig &middot; fixed camera</h2>',
        '<div class="rp-sub">One sweep fits where a bolted-down camera looks. Tap the tool tip '
        + "in one still, the arm drives eight poses and the stills are taken for you; the pose is "
        + "fitted and verified, and after that taps need no tag. <b>Handheld mode does not use "
        + "this card</b> &mdash; there the base tag sets the camera on every tap.</div>",
        '<div class="rp-kv">'];

      rows.push("<span>Arm</span><span>" + (info.execute
        ? (armState && !armState.ok
          ? (armState.refused
            ? 'will move, but <span class="rp-bad">the gateway refused /pose: '
            : 'will move, but <span class="rp-bad">not reachable: ')
            + escapeHtml(armState.error) + "</span>"
          : 'will move &middot; REBOT_EXECUTE=1 on the host and --allow-execute on the edge')
        : 'execution is off, so the sweep cannot run') + "</span>");
      rows.push("<span>Camera</span><span>" + (info.camera ? escapeHtml(info.camera)
        : "not configured (set REBOT_CAMERA)") + "</span>");
      rows.push("<span>Sweep</span><span>" + (info.sweep ? info.sweep.poses : 8)
        + " poses &middot; about "
        + (info.sweep && info.sweep.seconds ? Math.round(info.sweep.seconds) : 90)
        + " s of motion</span>");
      rows.push("<span>Current rig</span><span>" + (rig
        ? escapeHtml(rig.fingerprint || "?") + " · " + (rig.poses || 0) + " poses · "
          + (rig.predicted_mm == null ? "?" : Number(rig.predicted_mm).toFixed(1)) + " mm predicted"
        : "none fitted yet") + "</span>");
      rows.push("</div>");
      if (info.error) rows.push('<div class="rp-bad">' + escapeHtml(info.error) + "</div>");

      if (!running) {
        if (cameraMode !== "fixed") {
          rows.push('<div class="rp-sub">The page is set to <b>handheld + base tag</b>: switch '
            + "it in the <b>Camera</b> chip before fitting a rig for a bolted-down camera.</div>");
        }
        rows.push('<div class="rp-row"><button data-action="capture"'
          + (pending ? " disabled" : "") + ">"
          + (shot ? "Retake the still" : "Capture one pose") + "</button>"
          + '<span class="rp-sub">step 1</span></div>');
      }

      if (shot) {
        const point = shot.point;
        rows.push('<div class="rp-shot-wrap"><img class="rp-shot" src="' + escapeHtml(shot.url)
          + '" alt="captured pose">'
          + (point ? '<i class="rp-pin" style="left:' + (100 * point.u / shot.width).toFixed(2)
            + "%;top:" + (100 * point.v / shot.height).toFixed(2) + '%"></i>' : "")
          + "</div>");
        rows.push('<div class="rp-sub">' + (point
          ? "tip at (" + point.u.toFixed(0) + ", " + point.v.toFixed(0)
            + ") px &mdash; run the sweep, or tap again"
          : "step 2 &middot; tap the tool tip in the picture") + "</div>");
      }

      if (running) {
        const fraction = progress && progress.of ? Math.min(1, (progress.pose || 0) / progress.of) : 0;
        rows.push('<div class="rp-bar"><i style="width:' + (fraction * 100).toFixed(0) + '%"></i></div>');
        rows.push('<div class="rp-sub">' + progressText() + "</div>");
        rows.push('<div class="rp-sub">The arm is moving on its own; keep clear of the cell.</div>');
      } else {
        // With a base marker configured the tap is not needed: the tag's PnP
        // gives the camera, so the sweep can start from a bare "go".
        const hasMarker = Boolean(info.marker);
        const ready = info.execute && info.camera && (hasMarker || (shot && shot.point));
        let hint = "step 3";
        if (!info.camera) {
          hint = "no camera: set REBOT_CAMERA and restart capture";
        } else if (!info.execute) {
          hint = "execution is off: REBOT_EXECUTE=1 on the host, --allow-execute on the edge";
        } else if (hasMarker) {
          hint = "the base marker sets the camera; no tap needed";
        } else if (!shot) {
          hint = "capture a pose first";
        } else if (!shot.point) {
          hint = "tap the tool tip in the picture";
        }
        rows.push('<div class="rp-row"><button class="primary" data-action="run"'
          + (ready ? "" : " disabled") + ">"
          + (hasMarker ? "Run the sweep from the base marker" : "Run the sweep from this tap") + "</button>"
          + '<span class="rp-sub rp-run-hint' + (ready ? "" : " rp-warn") + '">'
          + escapeHtml(hint) + "</span></div>");
      }

      rows.push(resultHtml(result));
      rows.push('<div class="rp-row"><button data-action="close">'
        + (running ? "Hide" : "Close") + "</button></div>");

      overlay.innerHTML = '<div class="rp-card">' + rows.join("") + "</div>";
      const runButton = overlay.querySelector('[data-action="run"]');
      if (runButton) runButton.addEventListener("click", onRun);
      const captureButton = overlay.querySelector('[data-action="capture"]');
      if (captureButton) captureButton.addEventListener("click", capturePose);
      const picture = overlay.querySelector(".rp-shot");
      if (picture) picture.addEventListener("click", onPicture);
      overlay.querySelector('[data-action="close"]').addEventListener("click", hide);
    }

    function onPicture(event) {
      if (!shot) return;
      const picture = event.currentTarget;
      const rect = picture.getBoundingClientRect();
      if (!rect.width || !rect.height) return;
      const scale = rect.width / shot.width;
      const u = (event.clientX - rect.left) / scale;
      const v = (event.clientY - rect.top) / scale;
      if (u < 0 || v < 0 || u > shot.width || v > shot.height) return;
      shot.point = { u: Math.round(u * 100) / 100, v: Math.round(v * 100) / 100 };
      render();
    }

    async function refresh() {
      const next = await load();
      if (!next || !next.ok) return;
      status = next;
      progress = next.progress || progress;
      if (!next.running && running) {
        // A sweep this panel started is settled by its own POST; one it merely
        // adopted has to end here, or it would spin forever.
        if (posting) return;
        running = false;
        clearInterval(poll);
        poll = null;
      }
      render();
    }

    async function onRun() {
      const hasMarker = Boolean(status && status.marker);
      if (!hasMarker && (!shot || !shot.point)) return;
      const now = Date.now();
      if (now > armedUntil) {
        // Check the arm *before* arming: a gateway that cannot see the edge is
        // the usual reason a sweep goes nowhere, and it should be said now.
        const arm = await checkArm();
        if (!arm.ok) {
          status = Object.assign({}, status, {
            error: (arm.refused ? "the arm gateway refused /pose: " : "the arm is not reachable: ")
              + arm.error,
          });
          render();
          return;
        }
        armedUntil = now + 15000;
        // Re-query: the card may have been re-rendered while we waited, and an
        // event's `currentTarget` is null by then anyway.
        const current = overlay.querySelector('[data-action="run"]');
        if (!current) return;
        current.classList.add("armed");
        current.textContent = "Confirm: move the arm";
        const hint = overlay.querySelector(".rp-run-hint");
        if (hint) {
          hint.textContent = "click again to start";
          hint.className = "rp-sub rp-run-hint";
        }
        setTimeout(() => {
          if (Date.now() >= armedUntil && !running) render();
        }, 15200);
        return;
      }

      armedUntil = 0;
      running = true;
      posting = true;
      startedAt = Date.now();
      result = null;
      progress = { pose: 0, of: (status && status.sweep ? status.sweep.poses : 8), phase: "starting" };
      render();
      poll = setInterval(refresh, 1000);

      const body = hasMarker
        ? { confirm: true }
        : {
          confirm: true,
          capture: shot.id,
          u: shot.point.u,
          v: shot.point.v,
        };

      try {
        const answer = await postJson("/calibrate/extrinsics", body);
        result = answer.data;
        if (answer.ok) shot = null;
      } catch (error) {
        result = { ok: false, error: error.message };
      } finally {
        running = false;
        posting = false;
        progress = null;
        clearInterval(poll);
        poll = null;
        const next = await load();
        if (next && next.ok) status = next;
        render();
      }
    }

    async function launch() {
      overlay.hidden = false;
      status = await load();
      result = null;
      render();
      if (status && status.running && !poll) {
        running = true;
        startedAt = Date.now();
        progress = status.progress || null;
        poll = setInterval(refresh, 1000);
        render();
      }
    }

    function hide() {
      overlay.hidden = true;
    }

    // `?rig=1` opens it straight away: a bookmark on the workstation lands on
    // the one thing that has to be done before a tap means anything.
    if (new URLSearchParams(window.location.search).has("rig")) {
      Promise.resolve().then(launch);
    }

    return { open: launch, close: hide };
  }

  function create(options) {
    styleOnce();
    const video = options.video;
    const stage = options.stage;
    const status = options.onStatus || function () {};
    const token = options.token || (() => "");

    const marker = document.createElement("div");
    marker.className = "tp-marker";
    marker.hidden = true;
    document.body.appendChild(marker);

    const panel = document.createElement("div");
    panel.className = "tp-panel";
    panel.hidden = true;
    stage.appendChild(panel);

    let enabled = false;
    let busy = false;
    let point = null;
    let result = null;
    let armedUntil = 0;

    function tokenValue() {
      return typeof token === "function" ? token() : token;
    }

    function headers() {
      const value = tokenValue();
      return value ? { "X-Rebot-Token": value } : {};
    }

    function placeMarker() {
      if (!point || !video.videoWidth) {
        marker.hidden = true;
        return;
      }
      const rect = video.getBoundingClientRect();
      const width = video.videoWidth;
      const height = video.videoHeight;
      const scale = Math.min(rect.width / width, rect.height / height);
      const x = rect.left + (rect.width - width * scale) / 2 + point.u * scale;
      const y = rect.top + (rect.height - height * scale) / 2 + point.v * scale;
      marker.style.left = x + "px";
      marker.style.top = y + "px";
      marker.hidden = false;
    }

    function hidePanel() {
      panel.hidden = true;
      panel.innerHTML = "";
      result = null;
    }

    function format(point) {
      return "[" + point.map((value) => (value >= 0 ? "+" : "") + value.toFixed(3)).join(" ") + "] m";
    }

    function showResult(data) {
      result = data;
      const summary = data.plan && data.plan.summary;
      const lines = [];
      if (summary) {
        lines.push('<div class="tp-title">Target ' + format(summary.target) + "</div>");
        lines.push('<div class="tp-meta">' + summary.waypoints + " waypoints · "
          + summary.duration_s.toFixed(1) + " s · worst IK "
          + (summary.worst_ik_m * 1000).toFixed(1) + " mm · peak "
          + summary.peak_tau.toFixed(1) + " N·m</div>");
      }
      const buttons = [];
      if (data.execute && data.plan && data.plan.id) {
        buttons.push('<button class="primary" data-action="execute">Move the arm</button>');
      }
      if (data.preview) {
        buttons.push('<a href="' + data.preview + '" target="_blank" rel="noreferrer">'
          + "<button type=\"button\">Annotated frame</button></a>");
      }
      buttons.push('<button data-action="dismiss">Dismiss</button>');
      panel.innerHTML = lines.join("") + '<div class="tp-row">' + buttons.join("") + "</div>";
      panel.hidden = false;

      panel.querySelectorAll("button[data-action]").forEach((button) => {
        if (button.dataset.action === "dismiss") button.addEventListener("click", hidePanel);
        if (button.dataset.action === "execute") button.addEventListener("click", () => onExecute(button));
      });
    }

    function showError(message, preview) {
      panel.innerHTML = '<div class="tp-title tp-error">No plan</div>'
        + '<div class="tp-meta">' + escapeHtml(message) + "</div>"
        + '<div class="tp-row">'
        + (preview ? '<a href="' + preview + '" target="_blank" rel="noreferrer"><button type="button">Annotated frame</button></a>' : "")
        + '<button data-action="dismiss">Dismiss</button></div>';
      panel.hidden = false;
      panel.querySelector('[data-action="dismiss"]').addEventListener("click", hidePanel);
    }

    function escapeHtml(text) {
      return String(text == null ? "" : text).replace(/[&<>"']/g, (character) => ({
        "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;",
      }[character]));
    }

    async function grabFrame() {
      const canvas = document.createElement("canvas");
      canvas.width = video.videoWidth;
      canvas.height = video.videoHeight;
      canvas.getContext("2d").drawImage(video, 0, 0, canvas.width, canvas.height);
      const blob = await new Promise((resolve, reject) => {
        canvas.toBlob((value) => (value ? resolve(value) : reject(new Error("JPEG encode failed"))),
          "image/jpeg", 0.95);
      });
      return { blob: blob, width: canvas.width, height: canvas.height };
    }

    async function postJson(url, body) {
      const response = await fetch(url, {
        method: "POST",
        headers: Object.assign({ "Content-Type": "application/json" }, headers()),
        body: JSON.stringify(body),
      });
      let data = {};
      try {
        data = await response.json();
      } catch (error) {
        data = { ok: false, error: "server said " + response.status };
      }
      return { ok: response.ok, status: response.status, data: data };
    }

    async function planAt(tapped) {
      if (busy) return;
      busy = true;
      panel.hidden = true;
      try {
        status("capturing the frame under the tap", "");
        const frame = await grabFrame();

        const form = new FormData();
        form.append("meta", JSON.stringify({
          purpose: "plan",
          capturedAt: new Date().toISOString(),
          image_size: [frame.width, frame.height],
          tap: tapped,
          source: options.source || "tap",
          settings: video.videoWidth ? {
            width: video.videoWidth, height: video.videoHeight,
          } : {},
        }));
        form.append("frame", frame.blob, "frame.jpg");

        const upload = await fetch("/capture", {
          method: "POST", body: form, headers: headers(),
        });
        const saved = await upload.json().catch(() => ({}));
        if (!upload.ok) throw new Error(saved.error || ("upload failed: " + upload.status));
        status("capture " + saved.id + ", locating the target", "");

        const answer = await postJson("/plan", { capture: saved.id, mode: cameraMode });
        if (!answer.ok) {
          status(answer.data.error || "no plan", "bad");
          showError(answer.data.error || ("plan failed: " + answer.status), answer.data.preview);
          return;
        }
        const data = answer.data;
        status("plan ready: " + data.plan.summary.waypoints + " waypoints", "good");
        showResult(data);
        if (options.onResult) options.onResult(data);
      } catch (error) {
        status("tap failed: " + error.message, "bad");
        showError(error.message);
      } finally {
        busy = false;
      }
    }

    async function onExecute(button) {
      if (!result || !result.plan) return;
      const now = Date.now();
      if (now > armedUntil) {
        // Two steps on purpose: the second click is the one that moves steel.
        armedUntil = now + 5000;
        button.classList.add("armed");
        button.textContent = "Confirm: move now";
        setTimeout(() => {
          if (Date.now() >= armedUntil) {
            button.classList.remove("armed");
            button.textContent = "Move the arm";
          }
        }, 5200);
        return;
      }
      armedUntil = 0;
      button.disabled = true;
      button.textContent = "Moving...";
      status("executing " + result.plan.id + " on the arm", "warn");
      try {
        const answer = await postJson("/execute", { id: result.plan.id });
        if (!answer.ok) throw new Error(answer.data.error || ("execute failed: " + answer.status));
        status("arm finished: " + result.plan.id, "good");
        button.textContent = "Done";
        if (answer.data.log) {
          const logLine = document.createElement("div");
          logLine.className = "tp-meta";
          logLine.textContent = String(answer.data.log).split("\n").filter(Boolean).slice(-2).join(" · ");
          panel.appendChild(logLine);
        }
      } catch (error) {
        status("execute failed: " + error.message, "bad");
        button.disabled = false;
        button.textContent = "Move the arm";
        showError(error.message);
      }
    }

    function onPointer(event) {
      if (!enabled || busy) return;
      const tapped = videoPoint(video, event.clientX, event.clientY);
      if (!tapped) return;
      point = tapped;
      placeMarker();
      planAt(tapped);
    }

    stage.addEventListener("pointerdown", onPointer);
    window.addEventListener("resize", placeMarker);

    // Both pages get the rig panel from here: one button, one API, no page code.
    const rigPanel = createRig(options.token);

    return {
      enable() { enabled = true; },
      disable() { enabled = false; },
      clear() { point = null; marker.hidden = true; hidePanel(); },
      get point() { return point; },
      plan: planAt,
      rig: rigPanel,
    };
  }

  window.TapPlanner = { create: create, rig: createRig, setMode: setMode, getMode: getMode };
})();
