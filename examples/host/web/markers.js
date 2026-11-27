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
 * tap and a tap-and-drag can be told apart. In via mode a tap attaches a
 * pass-through point to the last marker -- on the way to it, not after it.
 */
(function () {
  "use strict";

  const videoTools = (typeof window !== "undefined" && window.RebotVideo) || null;
  const MAX_MARKERS = 10;
  const MAX_VIAS = 5;

  function reverseTargets(targets) {
    if (!Array.isArray(targets) || !targets.length) return [];
    const n = targets.length;
    const out = [];
    for (let i = 0; i < n; i += 1) {
      const origIdx = n - 1 - i;
      const src = targets[origIdx];
      const copy = Object.assign({}, src);
      delete copy.via;
      if (i > 0) {
        // Leg from original marker (origIdx + 1) back to origIdx traverses
        // (origIdx + 1)'s incoming vias in reverse order.
        const prevOrig = targets[origIdx + 1];
        if (prevOrig && Array.isArray(prevOrig.via) && prevOrig.via.length) {
          copy.via = prevOrig.via.slice().reverse();
        }
      }
      out.push(copy);
    }
    return out;
  }

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
    let viaMode = false;            // taps attach vias to the last marker
    let generation = 0;             // invalidates a locate that a clear overtook
    let projectionTimer = 0;

    function pixelOf(entry, tip) {
      if (projectPoint) {
        try {
          const live = projectPoint(tip);
          if (live && Number.isFinite(live[0]) && Number.isFinite(live[1])) {
            return live;
          }
        } catch (error) {
          // Fall back to the capture's own projection/tap.
        }
      }
      if (entry.pixel && Number.isFinite(entry.pixel[0]) && Number.isFinite(entry.pixel[1])) {
        return entry.pixel;
      }
      return [entry.u, entry.v];
    }

    function projectedPixel(marker) {
      return pixelOf(marker, marker.target.tip);
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
        // Amber for odd (close), green for even (open); a red outline only
        // marks a target outside the planner's reach gate.
        pin.className = "seq-pin " + ((index + 1) % 2 === 1 ? "half" : "open")
          + (marker.radius > 0.75 ? " far" : "")
          + (marker.adjusted ? " adjusted" : "");
        pin.style.left = point.x + "px";
        pin.style.top = point.y + "px";
        pin.textContent = String(index + 1);
        pin.title = (index + 1) + " · " + ((index + 1) % 2 === 1
          ? "close" : "open") + (marker.adjusted ? " · adjusted to reachable point" : "");
        container.appendChild(pin);
        // Via pins: dashed blue, lettered after their marker (1a, 1b, ...).
        // The arm passes through them without stopping, on the way to that
        // marker; they never change the gripper parity.
        (marker.vias || []).forEach((via, viaIndex) => {
          const viaPixel = pixelOf(via, via.tip);
          const viaPoint = videoTools.screenPoint(video, stage, viaPixel[0], viaPixel[1]);
          if (!viaPoint) return;
          const viaPin = document.createElement("div");
          viaPin.className = "seq-pin via";
          viaPin.style.left = viaPoint.x + "px";
          viaPin.style.top = viaPoint.y + "px";
          viaPin.textContent = (index + 1) + String.fromCharCode(97 + viaIndex);
          viaPin.title = "via " + String.fromCharCode(97 + viaIndex)
            + " to reach marker " + (index + 1) + " · pass-through";
          container.appendChild(viaPin);
        });
      });
      container.hidden = calibrating;
      container.classList.toggle("disabled", !enabled || busy);
      scheduleProjection();
    }

    async function addAt(clientX, clientY) {
      // Returns {ok, error?, index?, marker?} so a caller that is not a finger
      // -- the chat sidebar -- can tell the model why a marker was refused.
      if (!enabled) return { ok: false, error: "markers are off: switch the page to markers" };
      if (calibrating) return { ok: false, error: "calibration mode is on" };
      if (busy) return { ok: false, error: "a marker is still being located" };
      if (!locate) {
        status("marker locate is not available in this page", "bad");
        return { ok: false, error: "this page has no locator" };
      }
      if (markers.length >= MAX_MARKERS) {
        status("10 markers is the limit; clear one first", "warn");
        return { ok: false, error: MAX_MARKERS + " markers is the limit" };
      }
      const tapped = videoTools.videoPoint(video, clientX, clientY);
      if (!tapped) return { ok: false, error: "that point is outside the video frame" };
      // Freeze the badge height at the instant of the tap: the marker must
      // remember the height it was tapped at, not wherever the badge is later.
      const tapHeight = typeof heightAt === "function" ? Number(heightAt()) : NaN;

      const current = generation;
      busy = true;
      onChange();
      try {
        const target = await locate(tapped, Number.isFinite(tapHeight) ? tapHeight : 0);
        if (current !== generation) return { ok: false, error: "the markers were cleared" };
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
          (markers.length % 2 === 1 ? " · gripper will close" : " · gripper will open"),
          markers.length % 2 === 1 ? "warn" : "good");
        render();
        onChange();
        return { ok: true, index: markers.length, marker: markers[markers.length - 1] };
      } catch (error) {
        status("marker not added: " + error.message, "bad");
        return { ok: false, error: error.message };
      } finally {
        busy = false;
        render();
        onChange();
      }
    }

    async function addViaAt(clientX, clientY) {
      // A via attaches to the last placed marker as its destination: the arm
      // passes through it, without stopping, on the leg into that marker
      // (from the previous marker, or the start pose for marker 1). Same
      // locate and frozen tap height as a marker; it never counts as one.
      if (!enabled) return { ok: false, error: "markers are off: switch the page to markers" };
      if (calibrating) return { ok: false, error: "calibration mode is on" };
      if (busy) return { ok: false, error: "a marker is still being located" };
      if (!markers.length) {
        return { ok: false, error: "place a marker first: vias attach to the last marker" };
      }
      if (!locate) {
        status("marker locate is not available in this page", "bad");
        return { ok: false, error: "this page has no locator" };
      }
      const owner = markers[markers.length - 1];
      if ((owner.vias || []).length >= MAX_VIAS) {
        status(MAX_VIAS + " vias per marker is the limit", "warn");
        return { ok: false, error: MAX_VIAS + " vias per marker is the limit" };
      }
      const tapped = videoTools.videoPoint(video, clientX, clientY);
      if (!tapped) return { ok: false, error: "that point is outside the video frame" };
      const tapHeight = typeof heightAt === "function" ? Number(heightAt()) : NaN;

      const current = generation;
      busy = true;
      onChange();
      try {
        const target = await locate(tapped, Number.isFinite(tapHeight) ? tapHeight : 0);
        if (current !== generation) return { ok: false, error: "the markers were cleared" };
        if (!target || !Array.isArray(target.tip) || target.tip.length !== 3) {
          throw new Error("the locator returned no [x, y, z] tip point");
        }
        const pixel = Array.isArray(target.pixel) ? target.pixel : [tapped.u, tapped.v];
        if (!owner.vias) owner.vias = [];
        owner.vias.push({ u: tapped.u, v: tapped.v, pixel: pixel, tip: target.tip });
        // The wire form rides on the marker's own target object.
        owner.target.via = owner.vias.map((via) => via.tip);
        const point = target.tip.map((value) => Number(value).toFixed(3)).join(", ");
        const heightText = Number.isFinite(tapHeight)
          ? " · " + Math.round(tapHeight * 1000) + " mm up" : "";
        status("via " + owner.vias.length + " to reach marker " + markers.length
          + " at tip [" + point + "] m" + heightText + " · pass-through", "good");
        render();
        onChange();
        return { ok: true, index: markers.length, via: owner.vias.length, marker: owner };
      } catch (error) {
        status("via not added: " + error.message, "bad");
        return { ok: false, error: error.message };
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

    function updateTip(markerNumber, newTip) {
      // Snap a 1-based marker to a planner-verified reachable [x, y, z] tip.
      // Preserves `original` so Clear/Undo restores the user's tap first.
      const idx = Number(markerNumber) - 1;
      if (!Number.isInteger(idx) || idx < 0 || idx >= markers.length) {
        return { ok: false, error: "marker " + markerNumber + " does not exist" };
      }
      if (!Array.isArray(newTip) || newTip.length !== 3
          || !newTip.every((v) => Number.isFinite(Number(v)))) {
        return { ok: false, error: "adjusted tip must be finite [x, y, z]" };
      }
      const marker = markers[idx];
      const tip = [Number(newTip[0]), Number(newTip[1]), Number(newTip[2])];
      const previousTip = marker.target.tip.slice();
      if (!marker.original) {
        marker.original = {
          tip: previousTip,
          pixel: Array.isArray(marker.pixel) ? marker.pixel.slice() : null,
          u: marker.u,
          v: marker.v,
          radius: marker.radius,
        };
      }
      marker.target.tip = tip;
      marker.radius = Math.hypot(tip[0], tip[1]);
      const projected = projectPoint ? (function () {
        try {
          const px = projectPoint(tip);
          return (px && Number.isFinite(px[0]) && Number.isFinite(px[1])) ? px : null;
        } catch (_err) {
          return null;
        }
      })() : null;
      if (projected) {
        marker.pixel = projected;
        marker.target.pixel = projected;
        marker.u = projected[0];
        marker.v = projected[1];
      }
      marker.adjusted = true;
      render();
      onChange();
      return {
        ok: true,
        index: idx + 1,
        previousTip: previousTip,
        tip: tip,
        pixel: projected || marker.pixel,
      };
    }

    function clearLast() {
      // Undo in reverse order: if a marker was auto-adjusted to match IK advice,
      // restore its original tap first; otherwise remove the last via or marker.
      if (busy) return false;
      if (!markers.length) return false;
      generation += 1;
      for (let i = markers.length - 1; i >= 0; i -= 1) {
        if (markers[i].adjusted && markers[i].original) {
          const orig = markers[i].original;
          markers[i].target.tip = orig.tip.slice();
          markers[i].radius = orig.radius;
          if (orig.pixel) {
            markers[i].pixel = orig.pixel.slice();
            markers[i].target.pixel = orig.pixel.slice();
          }
          markers[i].u = orig.u;
          markers[i].v = orig.v;
          markers[i].adjusted = false;
          delete markers[i].original;
          render();
          onChange();
          return "adjustment on marker " + (i + 1);
        }
      }
      const owner = markers[markers.length - 1];
      let removed;
      if (owner.vias && owner.vias.length) {
        const viaIndex = owner.vias.length - 1;
        owner.vias.pop();
        if (owner.vias.length) owner.target.via = owner.vias.map((via) => via.tip);
        else delete owner.target.via;
        removed = "via " + String.fromCharCode(97 + viaIndex)
          + " of marker " + markers.length;
      } else {
        markers.pop();
        removed = "marker " + (markers.length + 1);
      }
      render();
      onChange();
      return removed;
    }

    return {
      addAt: addAt,
      addViaAt: addViaAt,
      updateTip: updateTip,
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
      setViaMode: function (value) {
        viaMode = Boolean(value);
      },
      // A caller that adds a marker without a gesture (the chat sidebar) can ask
      // for the pins to be laid out again once its own state has settled.
      redraw: function () { render(); },
      get count() { return markers.length; },
      get viaCount() {
        return markers.reduce((total, marker) => total + ((marker.vias || []).length), 0);
      },
      get viaMode() { return viaMode; },
      get targets() { return markers.map((marker) => marker.target); },
      get reversedTargets() { return reverseTargets(markers.map((marker) => marker.target)); },
      get busy() { return busy; },
    };
  }

  const api = {
    create: create,
    reverseTargets: reverseTargets,
    MAX_MARKERS: MAX_MARKERS,
    MAX_VIAS: MAX_VIAS,
  };
  if (typeof window !== "undefined") window.MarkerSequence = api;
  if (typeof module !== "undefined" && module.exports) module.exports = api;
})();
