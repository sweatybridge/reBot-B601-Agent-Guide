/* Numbered markers on the phone preview, and nothing else.
 *
 * A tap captures the frame under the finger, locates that pixel in the arm's
 * base frame through `POST /capture` + `POST /locate`, and keeps the target for
 * the sequence planner. Each pin is drawn at the projection of its base-frame
 * target, not at the raw finger pixel: the capture's pixel is the fallback, and
 * the reach overlay's live pose reprojects the pins as the phone moves. The
 * pins are DOM elements so they stay crisp and can be numbered; the overlay
 * canvas below them is untouched.
 *
 * The page owns what Start does. This module only answers: how many markers,
 * which targets, and is a locate still in flight?
 */
(function () {
  "use strict";

  const MAX_MARKERS = 10;

  function create(options) {
    const video = options.video;
    const stage = options.stage;
    const container = options.pins;
    const status = options.onStatus || function () {};
    const token = options.token || (() => "");
    const onChange = options.onChange || function () {};
    const projectPoint = options.project || null;

    const markers = [];
    let busy = false;
    let enabled = true;
    let calibrating = false;
    let generation = 0;             // invalidates a locate that a clear overtook
    let projectionTimer = 0;

    function headers() {
      const value = typeof token === "function" ? token() : token;
      return value ? { "X-Rebot-Token": value } : {};
    }

    async function api(path, request) {
      const request_ = Object.assign({ cache: "no-store" }, request || {});
      request_.headers = Object.assign({}, request_.headers || {}, headers());
      return fetch(path, request_);
    }

    function videoPoint(clientX, clientY) {
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

    function screenPoint(u, v) {
      const rect = video.getBoundingClientRect();
      const host = stage.getBoundingClientRect();
      const width = video.videoWidth;
      const height = video.videoHeight;
      if (!width || !height) return null;
      const scale = Math.min(rect.width / width, rect.height / height);
      return {
        x: rect.left - host.left + (rect.width - width * scale) / 2 + u * scale,
        y: rect.top - host.top + (rect.height - height * scale) / 2 + v * scale,
      };
    }

    function projectedPixel(marker) {
      if (projectPoint) {
        try {
          const live = projectPoint(marker.target.point);
          if (live && Number.isFinite(live[0]) && Number.isFinite(live[1])) {
            return live;
          }
        } catch (error) {
          // Fall back to the capture's own projection/tap.
        }
      }
      if (marker.pixel && Number.isFinite(marker.pixel[0]) && Number.isFinite(marker.pixel[1])) {
        return marker.pixel;
      }
      return [marker.u, marker.v];
    }

    function scheduleProjection() {
      if (projectionTimer || !markers.length || !projectPoint) return;
      projectionTimer = window.setTimeout(() => {
        projectionTimer = 0;
        if (markers.length && !calibrating) render();
      }, 33);
    }

    function render() {
      if (!container) return;
      container.innerHTML = "";
      markers.forEach((marker, index) => {
        const pixel = projectedPixel(marker);
        const point = screenPoint(pixel[0], pixel[1]);
        if (!point) return;
        const pin = document.createElement("div");
        // Amber for odd (half-close), green for even (open); a red outline only
        // marks a target outside the planner's reach gate.
        pin.className = "seq-pin " + ((index + 1) % 2 === 1 ? "half" : "open")
          + (marker.radius > 0.75 ? " far" : "");
        pin.style.left = point.x + "px";
        pin.style.top = point.y + "px";
        pin.textContent = String(index + 1);
        pin.title = (index + 1) + " · " + ((index + 1) % 2 === 1
          ? "half-close" : "open");
        container.appendChild(pin);
      });
      container.hidden = calibrating;
      container.classList.toggle("disabled", !enabled || busy);
      scheduleProjection();
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

    async function addAt(clientX, clientY) {
      if (!enabled || calibrating || busy) return;
      if (markers.length >= MAX_MARKERS) {
        status("10 markers is the limit; clear one first", "warn");
        return;
      }
      const tapped = videoPoint(clientX, clientY);
      if (!tapped) return;

      const current = generation;
      busy = true;
      onChange();
      try {
        const frame = await grabFrame();
        const form = new FormData();
        form.append("meta", JSON.stringify({
          purpose: "plan",
          capturedAt: new Date().toISOString(),
          image_size: [frame.width, frame.height],
          tap: tapped,
          source: "phone-sequence",
        }));
        form.append("frame", frame.blob, "frame.jpg");

        const upload = await api("/capture", { method: "POST", body: form });
        const saved = await upload.json().catch(() => ({}));
        if (!upload.ok) throw new Error(saved.error || ("upload failed: " + upload.status));

        const mode = window.TapPlanner ? window.TapPlanner.getMode() : "handheld";
        const response = await api("/locate", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ capture: saved.id, mode: mode }),
        });
        const data = await response.json().catch(() => ({}));
        if (!response.ok || !data.ok) {
          throw new Error(data.error || ("locate failed: " + response.status));
        }

        if (current !== generation) return;
        const target = data.target;
        const pixel = Array.isArray(target.pixel) ? target.pixel : [tapped.u, tapped.v];
        markers.push({ u: tapped.u, v: tapped.v, pixel: pixel, target: target,
          radius: Number(target.radius || 0) });
        const point = (target.point || []).map((value) => value.toFixed(3)).join(", ");
        status("marker " + markers.length + " at [" + point + "] m" +
          (markers.length % 2 === 1 ? " · gripper will half-close" : " · gripper will open"),
          markers.length % 2 === 1 ? "warn" : "good");
        render();
        onChange();
      } catch (error) {
        status("marker not added: " + error.message, "bad");
      } finally {
        busy = false;
        render();
        onChange();
      }
    }

    stage.addEventListener("pointerdown", (event) => {
      if (!enabled || calibrating || busy) return;
      addAt(event.clientX, event.clientY);
    });

    window.addEventListener("resize", render);
    video.addEventListener("loadedmetadata", render);
    video.addEventListener("resize", render);
    if (window.ResizeObserver) {
      new ResizeObserver(render).observe(video);
      new ResizeObserver(render).observe(stage);
    }

    function clearLast() {
      if (busy) return false;
      if (!markers.length) return false;
      generation += 1;
      markers.pop();
      render();
      onChange();
      return true;
    }

    return {
      addAt: addAt,
      clearLast: clearLast,
      clear: function () {
        generation += 1;
        markers.length = 0;
        render();
        onChange();
      },
      setEnabled: function (value) {
        enabled = Boolean(value);
        render();
      },
      setCalibrating: function (value) {
        calibrating = Boolean(value);
        render();
      },
      get count() { return markers.length; },
      get targets() { return markers.map((marker) => marker.target); },
      get busy() { return busy; },
    };
  }

  window.MarkerSequence = { create: create, MAX_MARKERS: MAX_MARKERS };
})();
