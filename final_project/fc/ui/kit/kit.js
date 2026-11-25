// Pupper FC UI kit: the pieces every screen shares. No build step, no dependencies.
//   h()            build DOM declaratively (no innerHTML strings, no escaping bugs)
//   patch()        replace a node's children only when its render key changed (cheap 10 Hz re-render)
//   pulse()        restart a one-shot animation attribute (role flip, score pop)
//   createStore()  one /state poller; screens subscribe
//   createOps()    the operations, from GET /api/ops plus screen-local view actions. Buttons, the key bar and
//                  the keyboard all come from it; enabled/label/next-args come from /state avail[op], so a key
//                  can never do something different from its button or from `fc.ctl op`.
//   createToast(), ask()   one-line feedback, one-question dialog
"use strict";
const SVG = new Set(["svg", "path", "circle", "use", "rect", "line", "polyline"]);
export function h(tag, props, ...kids) {
  const el = SVG.has(tag) ? document.createElementNS("http://www.w3.org/2000/svg", tag) : document.createElement(tag);
  for (const [k, v] of Object.entries(props || {})) {
    if (v == null || v === false) continue;
    if (k.startsWith("on")) el.addEventListener(k.slice(2), v);
    else if (k === "style" && typeof v === "object") Object.assign(el.style, v);
    else el.setAttribute(k, v === true ? "" : v);
  }
  for (const c of kids.flat(Infinity)) if (c != null && c !== false) el.append(c.nodeType ? c : document.createTextNode(String(c)));
  return el;
}
export function patch(el, key, render) {
  if (el.__key === key) return false;
  el.__key = key; el.replaceChildren(...[render()].flat(Infinity).filter((n) => n != null && n !== false)); return true;
}
export function pulse(el, attr) { if (!el) return; el.removeAttribute(attr); void el.offsetWidth; el.setAttribute(attr, ""); }
export const keyOf = (nodes) => [nodes].flat(Infinity).filter(Boolean).map((n) => n.outerHTML || String(n)).join("");
export const NAME = (id) => String(id ?? "").toUpperCase();
export const cap = (t) => { t = String(t || ""); return t.charAt(0).toUpperCase() + t.slice(1); };
export const clock = (s) => { s = Math.max(0, Math.ceil(s)); return `${Math.floor(s / 60)}:${String(s % 60).padStart(2, "0")}`; };

export const icon = (name) => h("svg", { viewBox: "0 0 24 24", "aria-hidden": "true" }, h("use", { href: `#i-${name}` }));
const ICONS = `<svg width="0" height="0" style="position:absolute" aria-hidden="true"><defs>
  <symbol id="i-eye" viewBox="0 0 24 24"><path d="M2 12s3.6-7 10-7 10 7 10 7-3.6 7-10 7S2 12 2 12z"/><circle cx="12" cy="12" r="3"/></symbol>
  <symbol id="i-ball" viewBox="0 0 24 24"><circle cx="12" cy="12" r="9"/><path d="M12 7l4 3-1.5 5h-5L8 10z"/></symbol>
  <symbol id="i-target" viewBox="0 0 24 24"><circle cx="12" cy="12" r="8"/><circle cx="12" cy="12" r="2"/><path d="M12 1v4M12 19v4M1 12h4M19 12h4"/></symbol>
  <symbol id="i-race" viewBox="0 0 24 24"><path d="M5 21V4M5 4h11l-2 4 2 4H5"/></symbol>
</defs></svg>`;
document.body.insertAdjacentHTML("afterbegin", ICONS);

// ---- store: one poller, many subscribers ----
export function createStore(url = "/state?lite=1", hz = 10) {
  const subs = new Set(); let state = null, at = 0, online = true;
  const tick = async () => {
    try { state = await (await fetch(url, { cache: "no-store" })).json(); at = performance.now(); online = true; }
    catch { online = false; }
    for (const f of subs) { try { f(state, online); } catch (e) { console.error("render", e); } }   // a render bug must not look like a dead app
    setTimeout(tick, online ? 1000 / hz : 300);
  };
  tick();
  return { subscribe: (f) => (subs.add(f), () => subs.delete(f)), get: () => state, at: () => at, online: () => online };
}

export function post(path, body) {
  return fetch(path, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) })
    .then((r) => r.json()).catch(() => ({ ok: false, error: "app not reachable" }));
}

// ---- toast and dialog ----
export function createToast() {
  const el = h("div", { class: "toast", role: "status" }); document.body.append(el);
  let t = 0;
  return (text, level) => { el.textContent = text; el.dataset.show = ""; if (level) el.dataset.level = level; else delete el.dataset.level;
    clearTimeout(t); t = setTimeout(() => delete el.dataset.show, 2400); };
}
const dlg = h("dialog", { class: "dialog" }); document.body.append(dlg);
dlg.addEventListener("cancel", (e) => e.preventDefault());       // Esc is E-stop, never "close dialog"
export function closeDialog() { if (dlg.open) dlg.close("no"); }
export function ask({ title, lines, yes, team }) {
  return new Promise((res) => {
    if (team) dlg.dataset.team = team; else delete dlg.dataset.team;
    const done = (v) => { dlg.close(); res(v); };
    dlg.onclose = () => res(false);
    dlg.replaceChildren(h("h3", null, title), h("ul", null, lines.map((l) => h("li", null, l))),
      h("div", { class: "acts" }, h("button", { class: "btn", autofocus: true, onclick: () => done(false) }, "Cancel"),
        h("button", { class: "btn", "data-tone": "go", onclick: () => done(true) }, yes)));
    dlg.showModal();
  });
}

// ---- operations ----
// Remote ops come from GET /api/ops. Local ops ({name, label, key, run, avail}) are view actions of one screen
// (switch camera, open a drawer); they never touch a robot. Several ops may share a key when their availability
// is mutually exclusive (the server guarantees this for P); the key runs whichever is enabled.
export async function createOps(store, toast) {
  const m = await (await fetch("/api/ops")).json();
  const ops = new Map(m.ops.map((o) => [o.name, o])), local = new Map(), byKey = new Map();
  for (const o of m.ops) if (o.key) byKey.set(o.key, [...(byKey.get(o.key) || []), o.name]);
  const api = {
    manifest: m, store, toast,
    get: (name) => ops.get(name) || local.get(name),
    local(o) { local.set(o.name, o); if (o.key) byKey.set(o.key, [...(byKey.get(o.key) || []), o.name]); return api; },
    avail(name) {
      const s = store.get(), lo = local.get(name);
      if (lo) { const a = lo.avail ? lo.avail(s) : {}; return { enabled: !a.reason, ...a }; }
      if (!s) return { enabled: false, reason: "app not reachable" };
      return (s.avail || {})[name] || { enabled: true };
    },
    // which op a key runs now: the enabled one, else the first (so its reason can be shown)
    forKey(key) { const names = byKey.get(key) || []; return names.find((n) => api.avail(n).enabled) || names[0]; },
    async run(name, args = {}, opts = {}) {
      const lo = local.get(name);
      if (lo) return lo.run(store.get(), args);
      const o = ops.get(name), a = api.avail(name);
      if (!o) return;
      if (!a.enabled) { toast(a.reason || `${o.label} is not available now`, "bad"); return { ok: false }; }
      if ((o.ask || opts.ask) && !opts.confirmed) {           // opts.ask: a more specific question from the caller
        const who = args.robot ? ` ${NAME(args.robot)}` : "", label = opts.label || a.label || o.label;
        if (!await ask({ title: `${label}${who}`, lines: (opts.ask || o.ask).split(/(?<=[.?:])\s+/), yes: `${label}${who}`,
                         team: (store.get().robots.find((r) => r.id === args.robot) || {}).team })) return { ok: false, cancelled: true };
        args = { ...args, confirm: true };
      }
      const r = await post("/api/op", { op: name, args: { ...(a.args || {}), ...args } });
      if (!r.ok) toast(r.error || "refused", "bad");
      else if (opts.done) toast(typeof opts.done === "function" ? opts.done(r) : opts.done, opts.level);
      if (r.warnings) toast(r.warnings.join("; "), "bad");
      return r;
    },
    // keyboard: capture phase, so Space and Esc win over every focused control and any open dialog.
    // A page whose inline safety block (FC_SAFETY_KEYS) ran first owns Space/Esc (fixed endpoints, no manifest needed); here they
    // only get their feedback toast.
    bindKeys(extra = {}) {
      if (window.FC_SAFETY_KEYS) window.FC_SAFETY_KEYS.done = (k, r) => {
        const o = extra.opts && extra.opts[k === "Space" ? "stop" : "estop"];
        if (!r || !r.ok) toast((r && r.error) || "app not reachable: use R3", "bad");
        else if (o && o.done) toast(o.done, o.level);
      };
      const name = (e) => e.code === "Space" ? "Space" : e.key === "Escape" ? "Esc" : e.key === "Tab" ? "Tab"
        : e.key === "Enter" ? "Enter" : e.key.startsWith("Arrow") ? e.key : e.key.length === 1 ? e.key.toUpperCase() : null;
      document.addEventListener("keydown", (e) => {
        const k = name(e); if (!k) return;
        const safety = k === "Space" || k === "Esc";
        if (safety && window.FC_SAFETY_KEYS) return;                       // already sent by the inline safety block
        if (!safety && (e.metaKey || e.ctrlKey || e.altKey || e.repeat || dlg.open)) return;   // a dialog owns its keys
        const op = api.forKey(k);
        if (!op) return;
        e.preventDefault(); e.stopPropagation();
        if (k === "Esc") closeDialog();
        api.run(op, e.shiftKey && extra.shift && extra.shift[op] ? extra.shift[op] : {}, extra.opts && extra.opts[op]);
      }, true);
      document.addEventListener("keyup", (e) => { if (e.code === "Space") e.preventDefault(); }, true);
    },
    // a prompt (key + label) bound to a key: label, enabled state and tone come from the registry every refresh
    prompt(key, extra) {
      const lbl = h("span", { class: "lbl" }), el = h("button", { class: "prompt", type: "button",
        onclick: (e) => { e.currentTarget.blur(); const op = api.forKey(key); if (op) api.run(op, e.shiftKey && el.shiftArgs ? el.shiftArgs : {}); } },
        h("span", { class: "key" }, key), lbl, extra || null);
      el.refresh = () => {
        const op = api.forKey(key), o = op && api.get(op), a = op ? api.avail(op) : { enabled: false };
        if (!o) return;
        const text = a.label || o.label; if (lbl.textContent !== text) lbl.textContent = text;
        el.setAttribute("aria-disabled", String(!a.enabled));
        if (a.state) el.dataset.state = a.state; else delete el.dataset.state;
        if (o.safety === "stop") el.dataset.tone = key === "Esc" ? "estop" : "stop";
        el.title = a.enabled ? o.doc || "" : a.reason || "";
      };
      return el;
    },
  };
  return api;
}
