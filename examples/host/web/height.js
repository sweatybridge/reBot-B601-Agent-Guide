/* Tap-and-drag height control for the phone viewport.
 *
 * A marker tap and a gesture that changes the arm's approach height both start
 * with a finger on the preview, so one owner has to arbitrate. A short press that
 * stays put is a tap: the page adds a marker there. Once the finger moves past
 * SLOP_PX the press becomes a height drag, up increases and down decreases
 * `hover_m`, the height of the tap plane above the worktop. The badge
 * in the top bar shows the current value.
 *
 * The module knows nothing about markers or the reach band. It reports the tap
 * and the height, and the page wires both.
 */
(function () {
  "use strict";

  const MIN_HEIGHT = 0.0;          // metres above the tapped worktop point
  const MAX_HEIGHT = 0.60;         // above this the arm model has no tool-down band
  const STEP = 0.005;              // 5 mm, the badge's resolution
  const SLOP_PX = 10;              // movement that turns a tap into a height drag
  const PER_PX = 0.0015;           // 100 px of drag is 15 cm

  function clampHeight(value, fallback, minimum, maximum) {
    const number = Number(value);
    if (!Number.isFinite(number)) return fallback;
    const low = Number.isFinite(minimum) ? minimum : MIN_HEIGHT;
    const high = Number.isFinite(maximum) ? maximum : MAX_HEIGHT;
    const snapped = Math.round(number / STEP) * STEP;
    return Math.max(low, Math.min(high, snapped));
  }

  function formatHeight(meters) {
    const centimetres = Math.round(Number(meters) * 1000) / 10;
    if (!Number.isFinite(centimetres)) return "-";
    return (Number.isInteger(centimetres) ? centimetres.toFixed(0) : centimetres.toFixed(1))
      + " cm";
  }

  function create(options) {
    const settings = options || {};
    const stage = settings.stage;
    const badge = settings.badge || null;
    const onTap = settings.onTap || function () {};
    const onChange = settings.onChange || function () {};
    const status = settings.onStatus || function () {};

    let minHeight = MIN_HEIGHT;
    let maxHeight = MAX_HEIGHT;
    let height = clampHeight(settings.initial == null ? 0.04 : settings.initial, 0.04,
                             minHeight, maxHeight);
    let touched = false;          // a drag has set the height; the host must not overwrite it
    let enabled = true;
    let pointerId = null;
    let mode = "idle";            // idle | pending | adjusting
    let startX = 0;
    let startY = 0;
    let startHeight = height;

    function render() {
      const label = formatHeight(height);
      if (!badge) return;
      badge.textContent = label;
      badge.setAttribute("aria-label", "Arm height " + label + " above the worktop");
    }

    function apply(value, live) {
      const next = clampHeight(value, height, minHeight, maxHeight);
      if (Math.abs(next - height) < 1e-9) return;
      height = next;
      render();
      onChange(height, Boolean(live));
    }

    function setAdjusting(value) {
      if (stage) stage.classList.toggle("adjusting", value);
      if (badge) badge.classList.toggle("adjusting", value);
    }

    function releasePointer() {
      if (pointerId !== null && stage) {
        try { stage.releasePointerCapture(pointerId); } catch (error) { /* not captured */ }
      }
    }

    function reset() {
      releasePointer();
      pointerId = null;
      mode = "idle";
      setAdjusting(false);
    }

    function startAdjust() {
      if (mode !== "pending") return;
      mode = "adjusting";
      touched = true;
      setAdjusting(true);
      if (navigator.vibrate) {
        try { navigator.vibrate(8); } catch (error) { /* not available */ }
      }
    }

    function down(event) {
      if (!enabled || mode !== "idle" || pointerId !== null) return;
      if (event.pointerType === "mouse" && event.button !== 0) return;
      if (event.isPrimary === false) return;
      pointerId = event.pointerId;
      mode = "pending";
      startX = event.clientX;
      startY = event.clientY;
      startHeight = height;
      if (stage) {
        try { stage.setPointerCapture(event.pointerId); } catch (error) { /* synthetic mouse */ }
      }
    }

    function move(event) {
      if (event.pointerId !== pointerId) return;
      if (mode === "pending") {
        if (Math.hypot(event.clientX - startX, event.clientY - startY) <= SLOP_PX) return;
        startAdjust();
      }
      if (mode !== "adjusting") return;
      event.preventDefault();
      apply(startHeight + (startY - event.clientY) * PER_PX, true);
    }

    function up(event) {
      if (event.pointerId !== pointerId) return;
      const finished = mode;
      const x = event.clientX;
      const y = event.clientY;
      reset();
      if (finished === "pending") {
        onTap(x, y);
      } else if (finished === "adjusting" && Math.abs(height - startHeight) > 1e-9) {
        status("arm height " + formatHeight(height) + " above the worktop", "good");
      }
    }

    function cancel(event) {
      if (event.pointerId !== pointerId) return;
      reset();
    }

    if (stage) {
      stage.addEventListener("pointerdown", down);
      // On window, not stage: a height drag often ends outside the video.
      window.addEventListener("pointermove", move, { passive: false });
      window.addEventListener("pointerup", up);
      window.addEventListener("pointercancel", cancel);
      window.addEventListener("blur", reset);
      stage.addEventListener("contextmenu", (event) => {
        if (mode !== "idle") event.preventDefault();
      });
    }
    render();

    return {
      get height() { return height; },
      get touched() { return touched; },
      get adjusting() { return mode === "adjusting"; },
      setHeight: function (value) {
        touched = true;
        apply(value, false);
      },
      adoptHeight: function (value) {
        if (!touched) apply(value, false);
      },
      setRange: function (minimum, maximum) {
        if (Number.isFinite(Number(minimum))) {
          minHeight = Math.max(MIN_HEIGHT, Number(minimum));
        }
        if (Number.isFinite(Number(maximum))) {
          maxHeight = Math.max(minHeight, Number(maximum));
        }
        apply(height, false);
      },
      setEnabled: function (value) {
        enabled = Boolean(value);
        if (!enabled) reset();
      },
    };
  }

  window.HeightControl = {
    create: create,
    format: formatHeight,
    MIN: MIN_HEIGHT,
    MAX: MAX_HEIGHT,
    STEP: STEP,
  };
})();
