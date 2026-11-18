/* Session trace queue for the /live sidebar (spec Phase 1, §4.3).
 *
 * Fire-and-forget by design (§8.4): events are batched (<=32 or 1 s) and POSTed
 * to /trace; a slow or down collector drops the batch, it never blocks the
 * chat loop. On pagehide the queue goes out with sendBeacon -- or fetch with
 * keepalive when a token is set, because sendBeacon cannot carry the
 * X-Rebot-Token header the server requires on writes.
 *
 * Frames are deduped by the sha256 of their decoded bytes: the first event
 * carries the data URL (the server extracts it into a content-addressed blob),
 * later events carry {"$blob": sha} so a frame shown twice uploads once.
 */
(function () {
  "use strict";

  const BATCH = 32;
  const FLUSH_MS = 1000;
  const FRAME_CACHE = 64;      // bounded: a long session must not pin every frame

  let session = "";
  let token = () => "";
  let queue = [];
  let timer = 0;
  let pseq = 0;
  const sentFrames = new Map();   // sha256 -> 1, insertion-ordered for eviction

  function init(options) {
    session = String((options && options.session) || "");
    token = (options && options.token) || (() => "");
    window.addEventListener("pagehide", flushOnHide);
  }

  function enqueue(event) {
    // pseq is page-monotonic and assigned here, at call time, so offline merge
    // can order page events even when a batch arrives late: sort (t, src, pseq).
    pseq += 1;
    event.pseq = pseq;
    event.t = new Date().toISOString();
    queue.push(event);
    if (queue.length >= BATCH) flush();
    else if (!timer) timer = setTimeout(flush, FLUSH_MS);
  }

  function trace(type, fields) {
    if (!session) return;
    enqueue(Object.assign({}, fields || {}, { type: type }));
  }

  function take() {
    if (timer) { clearTimeout(timer); timer = 0; }
    const batch = queue.slice(0, BATCH);
    queue = queue.slice(BATCH);
    if (queue.length && !timer) timer = setTimeout(flush, FLUSH_MS);
    return batch;
  }

  function headers(json) {
    const out = { "X-Rebot-Session": session };
    const value = typeof token === "function" ? token() : token;
    if (value) out["X-Rebot-Token"] = value;
    if (json) out["Content-Type"] = "application/json";
    return out;
  }

  function flush() {
    if (!session || !queue.length) { if (timer) { clearTimeout(timer); timer = 0; } return; }
    const batch = take();
    const body = JSON.stringify({ session: session, events: batch });
    // The answer is deliberately ignored: a dropped batch costs evidence, not
    // a chat turn. No retry queue -- a down collector must not grow memory.
    fetch("/trace", { method: "POST", headers: headers(true), body: body,
                      cache: "no-store", keepalive: true }).catch(() => {});
    if (queue.length) flush();
  }

  function flushOnHide() {
    if (!session || !queue.length) return;
    const batch = take();
    const body = JSON.stringify({ session: session, events: batch });
    const value = typeof token === "function" ? token() : token;
    if (!value && navigator.sendBeacon) {
      navigator.sendBeacon("/trace", new Blob([body], { type: "application/json" }));
      return;
    }
    // keepalive lets the request outlive the page; headers carry the token.
    fetch("/trace", { method: "POST", headers: headers(true), body: body,
                      cache: "no-store", keepalive: true }).catch(() => {});
  }

  async function sha256Hex(bytes) {
    const digest = await crypto.subtle.digest("SHA-256", bytes);
    return Array.from(new Uint8Array(digest))
      .map((b) => b.toString(16).padStart(2, "0")).join("");
  }

  function dataUrlBytes(url) {
    const base64 = url.slice(url.indexOf(",") + 1);
    const raw = atob(base64);
    const bytes = new Uint8Array(raw.length);
    for (let i = 0; i < raw.length; i += 1) bytes[i] = raw.charCodeAt(i);
    return bytes;
  }

  async function frameSha(url) {
    return sha256Hex(dataUrlBytes(url));
  }

  function rememberFrame(sha) {
    sentFrames.set(sha, 1);
    while (sentFrames.size > FRAME_CACHE) {
      sentFrames.delete(sentFrames.keys().next().value);
    }
  }

  /* Trace an event that carries a frame. The event is enqueued synchronously
   * (so its pseq keeps call order); the sha resolves in milliseconds, usually
   * long before the 1 s flush. A frame already uploaded becomes a $blob ref;
   * the first one keeps its data URL for the server to extract. If the flush
   * beats the digest the data URL is sent -- correct, just not deduped. */
  function traceFrame(type, fields, dataUrl) {
    if (!session) return;
    const event = Object.assign({}, fields || {}, { type: type, frame: dataUrl || null });
    enqueue(event);
    if (!dataUrl) return;
    frameSha(dataUrl).then((sha) => {
      if (sentFrames.has(sha)) event.frame = { $blob: sha };
      else rememberFrame(sha);
    }).catch(() => {});
  }

  window.RebotTrace = {
    init: init,
    trace: trace,
    traceFrame: traceFrame,
    flush: flush,
    sha256Hex: sha256Hex,
    frameSha: frameSha,
  };
})();