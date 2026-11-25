// Robot camera feeds. One component, two sources:
//   real robot with cam_url in robots.json -> /cam/<id>.jpg (robot/camstream.py on the Pi), ~8 fps, NO SIGNAL when stale
//   sim                                    -> a rendered robot's-eye view from the sim world, labelled SIM
// A robot with no camera gets no feed (nothing to show, nothing drawn).
//   const f = createRobotFeed(robot, { key: "1", onclick });  f.el;  f.update(state, { field: canvasOrNull })
// With `field` set the feed shows that canvas instead (the field moved here while this robot's camera is big).
import { h } from "/kit/kit.js";

export const hasFeed = (s, r) => s.source === "sim" || !!r.cam_url;

export function createRobotFeed(r, opt = {}) {
  const cv = h("canvas"), img = h("img", { alt: "" }), chips = h("div", { class: "chips" });
  const el = h("div", { class: "feedbox", "data-team": r.team, "data-click": opt.onclick ? true : null, onclick: opt.onclick }, cv, img, chips);
  let busy = false, last = 0, chipKey = "";
  const setChips = (items) => { const k = items.map((i) => i.outerHTML).join(""); if (k !== chipKey) { chipKey = k; chips.replaceChildren(...items); } };
  function update(s, view = {}) {
    const key = opt.key ? h("span", { class: "key" }, opt.key) : null;
    const name = view.big ? h("span", { class: "plate", "data-team": r.team, "data-fill": "team" }, h("span", { class: "display", style: { padding: ".15em .4em 0" } }, `${r.id.toUpperCase()} CAM`)) : null;
    if (view.field) {                                     // the field camera / map, while this robot's camera is big
      fit(cv); const g = cv.getContext("2d"), src = view.field, sc = Math.max(cv.width / src.width, cv.height / src.height);
      g.drawImage(src, (cv.width - src.width * sc) / 2, (cv.height - src.height * sc) / 2, src.width * sc, src.height * sc);
      cv.style.display = ""; img.style.display = "none"; el.dataset.signal = "ok";
      setChips([h("span", { class: "chip" }, s.source === "camera" ? "FIELD CAM" : "MAP"), key].filter(Boolean)); return;
    }
    if (s.source === "sim") {
      const ok = drawPOV(cv, r.id, s);
      cv.style.display = ""; img.style.display = "none"; el.dataset.signal = ok ? "ok" : "none";
      setChips([name, h("span", { class: "chip" }, "SIM VIEW"), key].filter(Boolean)); return;
    }
    const cam = (s.play.robots[r.id] || {}).cam, ok = !!(cam && cam.ok);
    cv.style.display = "none"; img.style.display = ""; el.dataset.signal = ok ? "ok" : "none";
    if (ok && !busy && performance.now() - last > 120) {   // poll the newest frame, one request at a time
      busy = true; last = performance.now();
      const n = new Image(); n.onload = () => { img.src = n.src; busy = false; }; n.onerror = () => { busy = false; };
      n.src = `/cam/${encodeURIComponent(r.id)}.jpg?t=${Date.now()}`;
    }
    setChips([name, ok ? h("span", { class: "chip", "data-live": true }, `${Math.round(cam.fps)} fps`) : null, key].filter(Boolean));
  }
  return { el, update };
}

function fit(cv) {
  const W = Math.round(cv.clientWidth * devicePixelRatio), H = Math.round(cv.clientHeight * devicePixelRatio);
  if (cv.width !== W || cv.height !== H) { cv.width = W; cv.height = H; }
  return W && H;
}

// ---- sim robot's-eye view: pinhole camera at the robot's pose (height CAM_H, pitched down) over the world model ----
const CAM_H = 0.12, PITCH = 14 * Math.PI / 180, HFOV = 62 * Math.PI / 180, NEAR = 0.04;
function drawPOV(cv, rid, S) {
  const w = S.world || { robots: {} }, me = w.robots[rid];
  if (!fit(cv) || !me) return false;
  const css = getComputedStyle(document.documentElement), KIT = { blue: css.getPropertyValue("--c-blue").trim(), red: css.getPropertyValue("--c-red").trim() };
  const W = cv.width, H = cv.height, g = cv.getContext("2d");
  const F = (W / 2) / Math.tan(HFOV / 2), c = Math.cos(me.yaw || 0), s = Math.sin(me.yaw || 0), cp = Math.cos(PITCH), sp = Math.sin(PITCH);
  const cam = (X, Y, Z) => { const dx = X - me.x, dy = Y - me.y, f = c * dx + s * dy, l = -s * dx + c * dy, u = Z - CAM_H;
    return [l, f * cp - u * sp, f * sp + u * cp]; };                              // [left, depth, up]
  const scr = ([l, d, v]) => [W / 2 - l / d * F, H / 2 - v / d * F];
  const clip = (pts) => { const out = [];                                         // near-plane clip, then project
    for (let i = 0; i < pts.length; i++) { const a = pts[i], b = pts[(i + 1) % pts.length];
      if (a[1] >= NEAR) out.push(a);
      if ((a[1] >= NEAR) !== (b[1] >= NEAR)) { const t = (NEAR - a[1]) / (b[1] - a[1]); out.push(a.map((q, k) => q + (b[k] - q) * t)); } }
    return out.map(scr); };
  const fill = (wp, col) => { const p = clip(wp.map((q) => cam(...q))); if (p.length < 3) return; g.beginPath(); p.forEach((q, i) => i ? g.lineTo(...q) : g.moveTo(...q)); g.closePath(); g.fillStyle = col; g.fill(); };
  const line = (a, b, col, wd) => { const p = clip([cam(...a), cam(...b)]); if (p.length < 2) return; g.beginPath(); g.moveTo(...p[0]); g.lineTo(...p[1]); g.strokeStyle = col; g.lineWidth = wd; g.stroke(); };
  const fl = S.field, L = fl.length / 2, Wd = fl.width / 2, G = fl.goal_width / 2, D = fl.goal_depth, A = 0.35;
  const dist = (x, y) => Math.hypot(x - me.x, y - me.y), hy = H / 2 - Math.tan(PITCH) * F;
  const sky = g.createLinearGradient(0, 0, 0, Math.max(1, hy)); sky.addColorStop(0, "#1b1f24"); sky.addColorStop(1, "#3a4048");
  g.fillStyle = sky; g.fillRect(0, 0, W, Math.max(0, hy)); g.fillStyle = "#5d6066"; g.fillRect(0, Math.max(0, hy), W, H);
  fill([[-L - A, -Wd - A, 0], [L + A, -Wd - A, 0], [L + A, Wd + A, 0], [-L - A, Wd + A, 0]], "#6b6e72");
  fill([[-L, -Wd, 0], [L, -Wd, 0], [L, Wd, 0], [-L, Wd, 0]], "#3d7a48");
  for (const [a, b] of [[[-L, -Wd], [L, -Wd]], [[L, -Wd], [L, Wd]], [[L, Wd], [-L, Wd]], [[-L, Wd], [-L, -Wd]], [[0, -Wd], [0, Wd]]])
    line([...a, 0], [...b, 0], "#e9eee6", Math.max(1.5, H / 160));
  [[[-L - A, Wd + A], [L + A, Wd + A]], [[L + A, Wd + A], [L + A, -Wd - A]], [[L + A, -Wd - A], [-L - A, -Wd - A]], [[-L - A, -Wd - A], [-L - A, Wd + A]]]
    .sort((a, b) => dist((b[0][0] + b[1][0]) / 2, (b[0][1] + b[1][1]) / 2) - dist((a[0][0] + a[1][0]) / 2, (a[0][1] + a[1][1]) / 2))
    .forEach(([a, b]) => fill([[...a, 0], [...b, 0], [...b, 0.28], [...a, 0.28]], "#8a8f96"));   // fence, far first
  for (const [sx, team] of [[-1, "blue"], [1, "red"]]) {                                           // goals in the keeper's kit
    const x0 = sx * L, x1 = sx * (L + D), wd = Math.max(3, H / 60);
    fill([[x1, -G, 0], [x1, G, 0], [x1, G, 0.16], [x1, -G, 0.16]], "#20262ecc");
    for (const y of [-G, G]) line([x0, y, 0], [x0, y, 0.16], KIT[team], wd);
    line([x0, -G, 0.16], [x0, G, 0.16], KIT[team], wd);
  }
  const things = [];
  if (w.ball) things.push([dist(w.ball.x, w.ball.y), () => {
    const p = cam(w.ball.x, w.ball.y, fl.ball_radius); if (p[1] < NEAR) return;
    const [x, y] = scr(p), r = fl.ball_radius / p[1] * F, bg = g.createRadialGradient(x - r * .35, y - r * .4, r * .1, x, y, r);
    bg.addColorStop(0, "#fff6c8"); bg.addColorStop(1, "#d39b00"); g.beginPath(); g.arc(x, y, r, 0, 6.283); g.fillStyle = bg; g.fill(); }]);
  for (const [oid, o] of Object.entries(w.robots)) if (oid !== rid) things.push([dist(o.x, o.y), () => {
    const team = (S.robots.find((q) => q.id === oid) || {}).team || "blue", oc = Math.cos(o.yaw || 0), os = Math.sin(o.yaw || 0);
    const box = (hx, hy2, z0, z1, col) => { const k = [[hx, hy2], [-hx, hy2], [-hx, -hy2], [hx, -hy2]].map(([u, v]) => [o.x + oc * u - os * v, o.y + os * u + oc * v]);
      k.map((p, i) => [p, k[(i + 1) % 4]]).sort((a, b) => dist(...b[0]) + dist(...b[1]) - dist(...a[0]) - dist(...a[1]))
        .forEach(([a, b]) => fill([[...a, z0], [...b, z0], [...b, z1], [...a, z1]], col));
      fill(k.map((p) => [...p, z1]), col); };
    box(0.15, 0.09, 0.05, 0.14, KIT[team]); box(0.14, 0.08, 0.14, 0.17, "#1d2228"); }]);
  things.sort((a, b) => b[0] - a[0]).forEach((t) => t[1]());
  const v = g.createRadialGradient(W / 2, H / 2, H * .3, W / 2, H / 2, W * .7); v.addColorStop(0, "#0000"); v.addColorStop(1, "#000a");
  g.fillStyle = v; g.fillRect(0, 0, W, H);
  return true;
}
