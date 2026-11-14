/* One /reach.json cache for the whole page.
 *
 * The reach overlay and the tap locator need the same camera calibration and
 * tag placement. They share this loader so a page load makes one request, and
 * fitting a new rig invalidates both at once.
 */
(function () {
  "use strict";

  if (typeof window === "undefined" || !window.RebotApi) {
    throw new Error("RebotApi is not loaded before reach-config.js");
  }

  let config = null;
  let loading = null;
  let generation = 0;

  function get() {
    return config;
  }

  function invalidate() {
    config = null;
    loading = null;
    generation += 1;
  }

  async function load(token) {
    if (config) return config;
    if (loading) return loading;
    const current = generation;
    loading = (async () => {
      const answer = await window.RebotApi.getJson("/reach.json", token);
      if (current !== generation) return load(token);
      if (!answer.ok) {
        throw new Error((answer.data && answer.data.error)
          || ("the host answered " + answer.status));
      }
      config = answer.data;
      return config;
    })();
    try {
      return await loading;
    } finally {
      loading = null;
    }
  }

  window.addEventListener("rebot:rig", invalidate);
  window.RebotConfig = { load: load, get: get, invalidate: invalidate };

  if (typeof module !== "undefined" && module.exports) {
    module.exports = { load: load, get: get, invalidate: invalidate };
  }
})();
