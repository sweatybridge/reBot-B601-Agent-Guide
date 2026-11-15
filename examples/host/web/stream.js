/* Parse the normalized Server-Sent Event stream from /agent.
 *
 * The transport is a POST, so EventSource cannot be used: the page reads
 * `response.body` and feeds decoded text here. The server emits whole frames
 * (`event: ...\n data: ...\n\n`), but a network chunk can split one anywhere,
 * so this keeps the tail until the terminating blank line arrives.
 */
(function () {
  "use strict";

  function create() {
    let buffer = "";

    function feed(text) {
      buffer += String(text || "").replace(/\r\n/g, "\n");
      const events = [];
      let cut;
      while ((cut = buffer.indexOf("\n\n")) >= 0) {
        const frame = buffer.slice(0, cut);
        buffer = buffer.slice(cut + 2);
        let event = "message";
        const data = [];
        for (const line of frame.split("\n")) {
          if (!line || line.startsWith(":")) continue;
          if (line.startsWith("event:")) {
            event = line.slice(6).trim();
          } else if (line.startsWith("data:")) {
            data.push(line.slice(5).replace(/^ /, ""));
          }
        }
        if (!data.length) continue;
        const payload = data.join("\n");
        if (payload.trim() === "[DONE]") {
          events.push({ event: "done", data: null });
          continue;
        }
        try {
          events.push({ event: event, data: JSON.parse(payload) });
        } catch (error) {
          events.push({
            event: "error",
            data: { error: "the model stream sent bad JSON: " + payload.slice(0, 120) },
          });
        }
      }
      return events;
    }

    function reset() {
      buffer = "";
    }

    return { feed: feed, reset: reset };
  }

  const api = { create: create };
  if (typeof window !== "undefined") window.RebotStream = api;
  if (typeof module !== "undefined" && module.exports) module.exports = api;
})();
