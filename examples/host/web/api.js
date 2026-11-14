/* Token-aware HTTP helpers for every page.
 *
 * The token can be a string or a function; callers keep ownership of where it
 * comes from. JSON responses are parsed defensively because a proxy or an older
 * server can answer with plain text.
 */
(function () {
  "use strict";

  function tokenValue(token) {
    return typeof token === "function" ? token() : token;
  }

  function headers(token) {
    const value = tokenValue(token);
    return value ? { "X-Rebot-Token": value } : {};
  }

  async function request(url, options, token) {
    const opts = Object.assign({}, options || {});
    opts.headers = Object.assign({}, opts.headers || {}, headers(token));
    return fetch(url, opts);
  }

  async function readJson(response) {
    const text = await response.text();
    if (!text) return {};
    try {
      return JSON.parse(text);
    } catch (error) {
      const said = text.trim().slice(0, 300);
      return { ok: false, error: said || ("server said " + response.status) };
    }
  }

  async function getJson(url, token, options) {
    try {
      const response = await request(url, Object.assign({ cache: "no-store" }, options || {}), token);
      const data = await readJson(response);
      return { ok: response.ok && data.ok !== false, status: response.status, data: data };
    } catch (error) {
      return { ok: false, status: 0, data: { ok: false, error: error.message } };
    }
  }

  async function postJson(url, body, token) {
    try {
      const response = await request(url, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(body || {}),
      }, token);
      const data = await readJson(response);
      return { ok: response.ok && data.ok !== false, status: response.status, data: data };
    } catch (error) {
      return { ok: false, status: 0, data: { ok: false, error: error.message } };
    }
  }

  async function postForm(url, form, token) {
    try {
      const response = await request(url, { method: "POST", body: form }, token);
      const data = await readJson(response);
      return { ok: response.ok && data.ok !== false, status: response.status, data: data };
    } catch (error) {
      return { ok: false, status: 0, data: { ok: false, error: error.message } };
    }
  }

  if (typeof window !== "undefined") {
    window.RebotApi = {
      headers: headers,
      request: request,
      getJson: getJson,
      postJson: postJson,
      postForm: postForm,
    };
  }
  if (typeof module !== "undefined" && module.exports) {
    module.exports = {
      headers: headers,
      request: request,
      getJson: getJson,
      postJson: postJson,
      postForm: postForm,
    };
  }
})();
