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
        status("capture " + saved.id + ", locating the tag", "");

        const answer = await postJson("/plan", { capture: saved.id });
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

    return {
      enable() { enabled = true; },
      disable() { enabled = false; },
      clear() { point = null; marker.hidden = true; hidePanel(); },
      get point() { return point; },
      plan: planAt,
    };
  }

  window.TapPlanner = { create: create };
})();
