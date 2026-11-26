#!/usr/bin/env node
/**
 * Upload a batch-validation repro metadata zip and individual stitched MP4 recordings
 * directly as native GitHub user-attachments onto a GitHub Issue via Chrome Remote Debugging
 * (chrome://inspect/#remote-debugging -> DevToolsActivePort).
 *
 * Requires Node.js 24+ (native WebSocket, zero npm dependencies).
 *
 * Usage:
 *   node skills/batch-validation/scripts/upload_issue_attachments.mjs \
 *     --issue 12 \
 *     --repo sweatybridge/reBot-B601-Agent-Guide \
 *     --manifest d:\...\batch_manifest.json \
 *     --export-dir d:\...\export_batch
 */
import fs from "node:fs";
import path from "node:path";

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

function parseArgs(argv) {
  const out = { repo: "sweatybridge/reBot-B601-Agent-Guide" };
  for (let i = 0; i < argv.length; i++) {
    const a = argv[i];
    if (a === "--issue") out.issue = argv[++i];
    else if (a === "--repo") out.repo = argv[++i];
    else if (a === "--manifest") out.manifest = argv[++i];
    else if (a === "--export-dir") out.exportDir = argv[++i];
  }
  if (!out.issue || !out.manifest || !out.exportDir) {
    console.error("Usage: upload_issue_attachments.mjs --issue <N> --manifest <json> --export-dir <dir> [--repo owner/name]");
    process.exit(2);
  }
  return out;
}

class CDP {
  constructor(wsUrl) {
    this.wsUrl = wsUrl;
    this.id = 0;
    this.pending = new Map();
    this.eventWaiters = new Map();
  }
  connect() {
    return new Promise((resolve, reject) => {
      const ws = new WebSocket(this.wsUrl);
      this.ws = ws;
      ws.addEventListener("open", () => resolve());
      ws.addEventListener("error", (e) => reject(new Error("WS error: " + (e.message || e.type))));
      ws.addEventListener("message", (ev) => {
        const msg = JSON.parse(ev.data);
        if (msg.id && this.pending.has(msg.id)) {
          const { res, rej } = this.pending.get(msg.id);
          this.pending.delete(msg.id);
          if (msg.error) rej(new Error("CDP error: " + JSON.stringify(msg.error)));
          else res(msg.result || {});
        } else if (msg.method && this.eventWaiters.has(msg.method)) {
          const list = this.eventWaiters.get(msg.method);
          if (list.length > 0) list.shift().res(msg.params || {});
        }
      });
    });
  }
  waitForEvent(method, timeoutMs = 20000) {
    return new Promise((res, rej) => {
      const timer = setTimeout(() => {
        const list = this.eventWaiters.get(method) || [];
        const idx = list.indexOf(entry);
        if (idx >= 0) list.splice(idx, 1);
        rej(new Error(`Timeout waiting for CDP event ${method}`));
      }, timeoutMs);
      const entry = { res: (v) => { clearTimeout(timer); res(v); } };
      if (!this.eventWaiters.has(method)) this.eventWaiters.set(method, []);
      this.eventWaiters.get(method).push(entry);
    });
  }
  send(method, params = {}, sessionId = undefined, timeoutMs = 90000) {
    const id = ++this.id;
    const payload = { id, method, params };
    if (sessionId) payload.sessionId = sessionId;
    return new Promise((res, rej) => {
      const timer = setTimeout(() => {
        this.pending.delete(id);
        rej(new Error(`CDP timeout on ${method}`));
      }, timeoutMs);
      this.pending.set(id, {
        res: (v) => { clearTimeout(timer); res(v); },
        rej: (e) => { clearTimeout(timer); rej(e); },
      });
      this.ws.send(JSON.stringify(payload));
    });
  }
  close() {
    this.ws?.close();
  }
}

async function evalJS(cdp, sid, expression, userGesture = false) {
  const r = await cdp.send("Runtime.evaluate", { expression, returnByValue: true, awaitPromise: true, userGesture }, sid);
  if (r.exceptionDetails) throw new Error("JS exception: " + JSON.stringify(r.exceptionDetails));
  return r.result?.value;
}

async function main() {
  const args = parseArgs(process.argv.slice(2));
  const manifest = JSON.parse(fs.readFileSync(args.manifest, "utf8"));
  const issueUrl = `https://github.com/${args.repo}/issues/${args.issue}`;

  const bundleName = manifest.bundle_name;
  const zipPath = path.join(args.exportDir, bundleName);
  if (!fs.existsSync(zipPath)) throw new Error("Missing metadata zip: " + zipPath);

  const files = [["zip", zipPath, bundleName]];
  for (const run of manifest.runs || []) {
    const slug = run.slug || `${run.key}-${run.exec_id}`;
    const mp4Name = slug + ".mp4";
    const mp4Path = path.join(args.exportDir, mp4Name);
    if (!fs.existsSync(mp4Path)) throw new Error("Missing recording: " + mp4Path);
    files.push([run.key, mp4Path, mp4Name]);
  }

  const portFile = path.join(process.env.LOCALAPPDATA, "Google", "Chrome", "User Data", "DevToolsActivePort");
  if (!fs.existsSync(portFile)) {
    throw new Error("Chrome DevToolsActivePort not found. Enable chrome://inspect/#remote-debugging in Chrome.");
  }
  const lines = fs.readFileSync(portFile, "utf8").trim().split(/\r?\n/);
  const wsUrl = `ws://127.0.0.1:${lines[0].trim()}${lines[1].trim()}`;
  console.log("Connecting to", wsUrl);

  const cdp = new CDP(wsUrl);
  await cdp.connect();
  console.log("Connected to Chrome CDP");

  let targetId = null;
  try {
    const created = await cdp.send("Target.createTarget", { url: issueUrl });
    targetId = created.targetId;
    const attached = await cdp.send("Target.attachToTarget", { targetId, flatten: true });
    const sid = attached.sessionId;
    await cdp.send("Page.enable", {}, sid);
    await cdp.send("Runtime.enable", {}, sid);
    await cdp.send("DOM.enable", {}, sid);

    for (let i = 0; i < 40; i++) {
      const ready = await evalJS(cdp, sid, "document.readyState === 'complete' && !!document.querySelector('h1, [data-testid=\"issue-title\"]')");
      if (ready) break;
      await sleep(500);
    }
    await sleep(1500);

    // Open Issue description editor via kebab ("...") -> Edit
    const openRes = await evalJS(cdp, sid, `
      (async () => {
        const hasPopulatedTa = () => Array.from(document.querySelectorAll('textarea')).some(t => (t.value || '').trim().length > 40);
        if (hasPopulatedTa()) return 'already_open';
        const kebabs = Array.from(document.querySelectorAll('button')).filter(b =>
          !!b.querySelector('svg.octicon-kebab-horizontal') ||
          /issue actions|body actions|more options/i.test(b.getAttribute('aria-label') || '')
        );
        for (const btn of kebabs) {
          btn.click();
          await new Promise(r => setTimeout(r, 450));
          const items = Array.from(document.querySelectorAll('[role="menuitem"], [role="menu"] button, [role="menu"] a, li button'));
          const editItem = items.find(el => el.textContent && el.textContent.trim() === 'Edit');
          if (editItem) {
            editItem.click();
            await new Promise(r => setTimeout(r, 800));
            if (hasPopulatedTa()) return 'clicked_kebab_edit';
          }
          document.body.click();
          await new Promise(r => setTimeout(r, 200));
        }
        return 'no_edit_found';
      })()
    `);
    console.log("Open editor:", openRes);
    await sleep(800);

    const taInfo = await evalJS(cdp, sid, `
      Array.from(document.querySelectorAll('textarea')).map((t, i) => ({
        i, len: (t.value || '').length, aria: t.getAttribute('aria-label')
      }))
    `);
    const bodyTa = (taInfo || []).find((x) => x.len > 40);
    if (!bodyTa) throw new Error("Could not open issue body editor (signed into GitHub in Chrome?). Textareas: " + JSON.stringify(taInfo));
    const taIdx = bodyTa.i;
    const origBody = await evalJS(cdp, sid, `document.querySelectorAll('textarea')[${taIdx}].value`);

    await cdp.send("Page.setInterceptFileChooserDialog", { enabled: true }, sid);
    const urlRegex = /https:\/\/github\.com\/user-attachments\/(?:files|assets)\/[^\s)"'>]+/g;
    const uploaded = {};

    for (const [key, fpath, fname] of files) {
      const beforeVal = await evalJS(cdp, sid, `document.querySelectorAll('textarea')[${taIdx}].value`);
      const beforeSet = new Set(beforeVal.match(urlRegex) || []);
      console.log(`Uploading ${key}: ${fname} (${fs.statSync(fpath).size} bytes)...`);

      const chooserPromise = cdp.waitForEvent("Page.fileChooserOpened", 20000);
      const rect = await evalJS(cdp, sid, `
        (() => {
          const ta = document.querySelectorAll('textarea')[${taIdx}];
          let root = ta.parentElement;
          for (let k = 0; k < 12 && root; k++) {
            const btns = Array.from(root.querySelectorAll('button, label'));
            const attachBtn = btns.find(b => /paste, drop, or click to add files|attach files|add files/i.test((b.textContent || '') + ' ' + (b.getAttribute('aria-label') || '')));
            if (attachBtn) {
              attachBtn.scrollIntoView({ block: 'center' });
              const r = attachBtn.getBoundingClientRect();
              return { x: r.left + r.width / 2, y: r.top + r.height / 2 };
            }
            root = root.parentElement;
          }
          throw new Error('Attach files button not found');
        })()
      `);
      await cdp.send("Input.dispatchMouseEvent", { type: "mousePressed", x: rect.x, y: rect.y, button: "left", clickCount: 1 }, sid);
      await cdp.send("Input.dispatchMouseEvent", { type: "mouseReleased", x: rect.x, y: rect.y, button: "left", clickCount: 1 }, sid);
      const chooser = await chooserPromise;
      await cdp.send("DOM.setFileInputFiles", { backendNodeId: chooser.backendNodeId, files: [fpath] }, sid);

      let found = null;
      for (let t = 0; t < 180; t++) {
        await sleep(1000);
        const curVal = await evalJS(cdp, sid, `document.querySelectorAll('textarea')[${taIdx}].value`);
        const newLinks = (curVal.match(urlRegex) || []).filter((u) => !beforeSet.has(u));
        if (newLinks.length > 0 && !curVal.includes("Uploading ")) {
          found = newLinks[0];
          console.log(`  -> ${key}: ${found}`);
          break;
        }
      }
      if (!found) throw new Error("Upload timed out for " + key);
      uploaded[key] = found;
    }

    const runSections = (manifest.runs || []).map((run, idx) => {
      const label = run.summary_label || run.strategy || run.key;
      return `#### Run ${idx + 1} — \`${run.exec_id}\` (${label})\n\n${uploaded[run.key]}`;
    }).join("\n\n");

    const appended = `\n\n---\n\n### Repro Data Bundle (JSONs, Stills & Contact Sheets)\n\n[${bundleName}](${uploaded.zip})\n\n---\n\n### Source Recordings\n\n${runSections}\n`;
    const finalBody = origBody.trimEnd() + appended;

    await evalJS(cdp, sid, `
      (() => {
        const ta = document.querySelectorAll('textarea')[${taIdx}];
        const setter = Object.getOwnPropertyDescriptor(window.HTMLTextAreaElement.prototype, 'value').set;
        setter.call(ta, ${JSON.stringify(finalBody)});
        ta.dispatchEvent(new Event('input', { bubbles: true }));
        ta.dispatchEvent(new Event('change', { bubbles: true }));
      })()
    `);
    await sleep(600);

    const saveRes = await evalJS(cdp, sid, `
      (() => {
        const ta = document.querySelectorAll('textarea')[${taIdx}];
        let root = ta.parentElement;
        for (let k = 0; k < 12 && root; k++) {
          const btn = Array.from(root.querySelectorAll('button')).find(b => /^(Save|Update comment|Update)$/i.test((b.textContent || '').trim()) && !b.disabled);
          if (btn) { btn.click(); return 'clicked: ' + btn.textContent.trim(); }
          root = root.parentElement;
        }
        return 'save_not_found';
      })()
    `);
    console.log("Save:", saveRes);
    await sleep(3000);
    console.log("UPLOADED_JSON:", JSON.stringify(uploaded, null, 2));
  } finally {
    if (targetId) {
      try { await cdp.send("Target.closeTarget", { targetId }); } catch {}
    }
    cdp.close();
  }
}

main().catch((e) => {
  console.error("FATAL:", e);
  process.exit(1);
});
