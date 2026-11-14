/* Numbered markers on the phone preview, and nothing else.
 *
 * A tap asks `locate.js` for the tool-tip position in the arm base frame, using
 * the camera pose the page already has (fixed rig or the base tag in that frame).
 * No frame is uploaded per marker. Each pin is drawn at the projection of its
 * base-frame tip target, not at the raw finger pixel; the reach overlay's live
 * pose reprojects the pins as the phone moves. The pins are DOM elements so they
 * stay crisp and can be numbered; the overlay canvas below them is untouched.
 *
 * The page owns what Start does. This module only answers: how many markers,
 * which targets, and is a locate still in flight? It also no longer listens for
 * pointerdown itself: the page routes a tap here from the height control, so a
 * tap and a tap-and-drag can be told apart.
 */
(function () {
  "use strict";

  const videoTools = (typeof window !== "undefined" && window.RebotVideo) || null;
  const MAX_MARKERS = 10;

  function create(options) {
    const video = options.video;
    const stage = options.stage;
    const container = options.pins;
    const status = options.onStatus || function () {};
    const onChange = options.onChange || function () {};
    const projectPoint = options.project || null;
    const heightAt = options.height || null;
    const locate = options.locate || null;

    const markers = [];
    let busy = false;
    let enabled = true;
    let calibrating = false;
    let generation = 0;             // invalidates a locate that a clear overtook
    let projectionTimer = 0;

    function projectedPixel(marker) {
      if (projectPoint) {
        try {
          const live = projectPoint(marker.target.tip);
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
        const point = videoTools.screenPoint(video, stage, pixel[0], pixel[1]);
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

    async function addAt(clientX, clientY) {
      if (!enabled || calibrating || busy) return;
      if (!locate) {
        status("marker locate is not available in this page", "bad");
        return;
      }
      if (markers.length >= MAX_MARKERS) {
        status("10 markers is the limit; clear one first", "warn");
        return;
      }
      const tapped = videoTools.videoPoint(video, clientX, clientY);
      if (!tapped) return;
      // Freeze the badge height at the instant of the tap: the marker must
      // remember the height it was tapped at, not wherever the badge is later.
      const tapHeight = typeof heightAt === "function" ? Number(heightAt()) : NaN;

      const current = generation;
      busy = true;
      onChange();
      try {
        const target = await locate(tapped, Number.isFinite(tapHeight) ? tapHeight : 0);
        if (current !== generation) return;
        if (!target || !Array.isArray(target.tip) || target.tip.length !== 3) {
          throw new Error("the locator returned no [x, y, z] tip point");
        }
        const pixel = Array.isArray(target.pixel) ? target.pixel : [tapped.u, tapped.v];
        const radius = Math.hypot(Number(target.tip[0]), Number(target.tip[1]));
        markers.push({ u: tapped.u, v: tapped.v, pixel: pixel, target: target,
          radius: radius });
        const point = target.tip.map((value) => Number(value).toFixed(3)).join(", ");
        const heightText = Number.isFinite(tapHeight)
          ? " · " + Math.round(tapHeight * 1000) + " mm up" : "";
        status("marker " + markers.length + " at tip [" + point + "] m" + heightText +
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
