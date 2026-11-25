// The overhead view: field (drawn pitch, or the field camera image with the calibrated overlays), robots,
// ball, targets, plans, predicted ball path. One canvas, redrawn every animation frame.
//
//   const pitch = createPitch(canvas, { state: () => S, view: () => "camera" | "map" });
//   pitch.goal("blue")       // ripple out of the net that blue scored into
//   pitch.camOk()            // true while the field camera picture is arriving
//
// Motion is snapshot interpolation: each object keeps its last timestamped observations (server clock minus
// the observation's age) and a playhead ~0.11 s behind the newest one, so 10 Hz state draws smoothly at 60 fps.
// Past the newest sample it extrapolates at most 0.15 s, then holds: a robot that is not seen stops on screen.
// Colours come from the CSS tokens, so the canvas and the HTML always agree.

const D = 0.11, EXTRAP = 0.15, TRAIL_S = 1.4, PRED_H = 3.0, PRED_STEP = 0.1, TAU = Math.PI * 2;

export function createPitch(cv, opts) {
  const css = getComputedStyle(document.documentElement), tok = (n) => css.getPropertyValue(n).trim();
  const COL = { blue: tok("--c-blue"), red: tok("--c-red") }, SOFT = { blue: tok("--c-blue-soft"), red: tok("--c-red-soft") };
  const DEEP = { blue: tok("--c-blue-deep"), red: tok("--c-red-deep") }, VOLT = tok("--c-volt"), CHALK = tok("--c-ink");
  const DISPLAY = tok("--f-display"), CALM = matchMedia("(prefers-reduced-motion: reduce)").matches;
  let cx = cv.getContext("2d"), S = null, M = null, goalFx = null, last = performance.now();
  const tracks = {}, trails = {};
  const dpr = () => devicePixelRatio || 1;
  const teamOf = (rid) => ((S.robots.find((q) => q.id === rid)) || {}).team || "blue";

  // ---- motion ----
  function feed(key, ts, o) {
    let tr = tracks[key];
    if (!tr) tr = tracks[key] = { s: [], ph: null, at: 0 };
    const s = tr.s, l = s[s.length - 1];
    if (l && ts <= l.ts + 1e-4) return;                                             // same observation again
    if (l && Math.hypot(o.x - l.x, o.y - l.y) > 0.8) { s.length = 0; tr.ph = null; }   // teleport: no slide
    s.push({ ts, x: o.x, y: o.y, yaw: o.yaw }); if (s.length > 10) s.shift();
    tr.at = performance.now() / 1000;
  }
  const angLerp = (a, b, u) => a + Math.atan2(Math.sin(b - a), Math.cos(b - a)) * u;
  function sample(key, nowS, dt) {
    const tr = tracks[key]; if (!tr || !tr.s.length) return null;
    const s = tr.s, n = s[s.length - 1], want = n.ts + (nowS - tr.at) - D;
    if (tr.ph === null || Math.abs(want - tr.ph) > 0.4) tr.ph = want;
    else tr.ph += dt + (want - tr.ph) * Math.min(1, dt * 5);
    const t = tr.ph;
    if (s.length === 1 || t <= s[0].ts) return { ...s[0] };
    for (let i = 1; i < s.length; i++) {
      if (t <= s[i].ts) {
        const a = s[i - 1], b = s[i], u = (t - a.ts) / Math.max(1e-6, b.ts - a.ts);
        return { x: a.x + (b.x - a.x) * u, y: a.y + (b.y - a.y) * u, yaw: a.yaw === undefined ? undefined : angLerp(a.yaw, b.yaw, u) };
      }
    }
    const p = s[s.length - 2], g = Math.min(t - n.ts, EXTRAP) / Math.max(1e-3, n.ts - p.ts);
    return { x: n.x + (n.x - p.x) * g, y: n.y + (n.y - p.y) * g,
             yaw: n.yaw === undefined ? undefined : angLerp(n.yaw, n.yaw + Math.atan2(Math.sin(n.yaw - p.yaw), Math.cos(n.yaw - p.yaw)), g) };
  }
  let ingested = null;
  function ingest(s) {
    if (s === ingested) return; ingested = s;
    const w = s.world || { robots: {} }, T = s.truth, keys = new Set();
    for (const [rid, o] of Object.entries(w.robots || {})) { feed("w:" + rid, s.t - (o.age || 0), o); keys.add("w:" + rid); }
    if (w.ball) { feed("wb", s.t - (w.ball.age || 0), w.ball); keys.add("wb"); }
    if (T) {
      for (const [rid, o] of Object.entries(T.robots || {})) { feed("t:" + rid, s.t, o); keys.add("t:" + rid); }
      if (T.ball) { feed("tb", s.t, T.ball); keys.add("tb"); }
    }
    for (const k of Object.keys(tracks)) if (!keys.has(k)) delete tracks[k];
  }

  // ---- field camera picture ----
  let camImg = null, camBusy = false, camT = 0, camOk = false;
  const camOn = () => opts.view() === "camera" && S && S.source === "camera" && camOk;
  function pollCam() {
    if (!S || S.source !== "camera" || camBusy || performance.now() - camT < 66) return;
    camBusy = true; camT = performance.now();
    const n = new Image();
    n.onload = () => { camImg = n; camOk = true; camBusy = false; };
    n.onerror = () => { camOk = false; camBusy = false; camT += 900; };
    n.src = "/view.jpg?t=" + Date.now();
  }

  // ---- mapping field (x, y) -> canvas px ----
  function mapping(W, H) {
    const F = S.field, ov = S.play.overlay;
    if (camOn() && camImg) {
      const ar = camImg.naturalWidth / camImg.naturalHeight, s = Math.min(W / ar, H);
      const iw = s * ar, ih = s, ix = (W - iw) / 2, iy = (H - ih) / 2;
      const hp = (h, x, y) => { const w = h[2][0] * x + h[2][1] * y + h[2][2];
        return [ix + iw * (h[0][0] * x + h[0][1] * y + h[0][2]) / w, iy + ih * (h[1][0] * x + h[1][1] * y + h[1][2]) / w]; };
      const p = (x, y, tag) => ov ? hp(tag ? ov.tag : ov.floor, x, y) : [NaN, NaN];
      const a = p(0, 0), b = p(0.5, 0);
      return { cam: true, img: [ix, iy, iw, ih], ok: !!ov, p, s: ov ? 2 * Math.hypot(b[0] - a[0], b[1] - a[1]) : 100 };
    }
    const m = 0.36, s = Math.min(W / (F.length + 2 * m), H / (F.width + 2 * m));
    return { cam: false, ok: true, s, p: (x, y) => [W / 2 + x * s, H / 2 - y * s] };
  }
  const tagged = (pts, tag) => { pts.tag = tag; return pts; };
  function poly(pts, close = true) { cx.beginPath(); pts.forEach(([x, y], i) => i ? cx.lineTo(...M.p(x, y, pts.tag)) : cx.moveTo(...M.p(x, y, pts.tag))); if (close) cx.closePath(); }
  const circlePts = (x, y, r, n = 28) => Array.from({ length: n }, (_, i) => [x + r * Math.cos(i / n * TAU), y + r * Math.sin(i / n * TAU)]);
  const px = (m) => m * M.s;   // metres -> canvas px (approximate on the camera image)

  // ---- the drawn pitch, cached offscreen per size ----
  let pitchCache = null, pitchKey = "";
  function drawPitch(W, H) {
    const F = S.field, L = F.length / 2, Wd = F.width / 2, G = F.goal_width / 2, Dp = F.goal_depth, [ox, oy] = M.p(0, 0);
    const bg = cx.createRadialGradient(ox, oy, 0, ox, oy, Math.max(W, H) * .7);
    bg.addColorStop(0, "#123049"); bg.addColorStop(1, "#08131f"); cx.fillStyle = bg; cx.fillRect(0, 0, W, H);
    const ap = 0.16;
    cx.fillStyle = "#1a5a35"; poly([[-L - ap, Wd + ap], [L + ap, Wd + ap], [L + ap, -Wd - ap], [-L - ap, -Wd - ap]]); cx.fill();
    for (let i = 0, n = 12; i < n; i++) {
      const x0 = -L + i * 2 * L / n;
      cx.fillStyle = i % 2 ? "#1f6b3e" : "#237546";
      poly([[x0, Wd], [x0 + 2 * L / n + 0.002, Wd], [x0 + 2 * L / n + 0.002, -Wd], [x0, -Wd]]); cx.fill();
    }
    for (const [sx, sy] of [[-1, 1], [1, 1], [-1, -1], [1, -1]]) {   // floodlight pools
      const [lx, ly] = M.p(sx * L * .8, sy * Wd * .8), g = cx.createRadialGradient(lx, ly, 0, lx, ly, px(1.3));
      g.addColorStop(0, "#ffffff10"); g.addColorStop(1, "#ffffff00"); cx.fillStyle = g; cx.fillRect(0, 0, W, H);
    }
    cx.strokeStyle = "#c7d3df55"; cx.lineWidth = Math.max(2, px(0.02));
    poly([[-L - ap, Wd + ap], [L + ap, Wd + ap], [L + ap, -Wd - ap], [-L - ap, -Wd - ap]]); cx.stroke();
    for (const [sx, team] of [[-1, "blue"], [1, "red"]]) {          // goals: net + posts in the keeper's kit
      const box = [[sx * L, G], [sx * (L + Dp), G], [sx * (L + Dp), -G], [sx * L, -G]];
      cx.fillStyle = "#0b1726"; poly(box); cx.fill();
      cx.fillStyle = COL[team] + "40"; poly(box); cx.fill();
      cx.save(); poly(box); cx.clip(); cx.strokeStyle = "#ffffff26"; cx.lineWidth = 1;
      for (let k = -2; k <= 2; k += 0.06) { poly([[sx * L, -G + k], [sx * (L + Dp), -G + k + Dp * 1.2]], false); cx.stroke(); poly([[sx * L, G - k], [sx * (L + Dp), G - k - Dp * 1.2]], false); cx.stroke(); }
      cx.restore();
      cx.strokeStyle = CHALK + "aa"; cx.lineWidth = Math.max(1.5, px(0.012)); poly(box.slice(), false); cx.stroke();
      cx.strokeStyle = COL[team]; cx.lineWidth = Math.max(4, px(0.035)); cx.lineCap = "round"; poly([[sx * L, G], [sx * L, -G]], false); cx.stroke(); cx.lineCap = "butt";
    }
    cx.strokeStyle = CHALK + "e6"; cx.lineWidth = Math.max(2, px(0.018)); cx.shadowColor = "#ffffff55"; cx.shadowBlur = 6 * dpr();
    poly([[-L, Wd], [L, Wd], [L, -Wd], [-L, -Wd]]); cx.stroke();
    poly([[0, Wd], [0, -Wd]], false); cx.stroke();
    poly(circlePts(0, 0, 0.25, 64)); cx.stroke();
    cx.shadowBlur = 0; cx.fillStyle = CHALK; poly(circlePts(0, 0, 0.018, 12)); cx.fill();
    const v = cx.createRadialGradient(ox, oy, Math.min(W, H) * .35, ox, oy, Math.max(W, H) * .75);
    v.addColorStop(0, "#00000000"); v.addColorStop(1, "#00000070"); cx.fillStyle = v; cx.fillRect(0, 0, W, H);
  }
  function drawField(W, H, now) {
    const F = S.field, L = F.length / 2, Wd = F.width / 2, G = F.goal_width / 2, Dp = F.goal_depth;
    if (M.cam) {
      cx.drawImage(camImg, ...M.img);
      if (!M.ok) return;
      cx.strokeStyle = "#ffffff66"; cx.lineWidth = 1.5 * dpr();
      poly([[-L, Wd], [L, Wd], [L, -Wd], [-L, -Wd]]); cx.stroke();
      poly([[0, Wd], [0, -Wd]], false); cx.stroke(); poly(circlePts(0, 0, 0.25, 48)); cx.stroke();
      for (const [sx, team] of [[-1, "blue"], [1, "red"]]) {
        cx.fillStyle = COL[team] + "38"; poly([[sx * L, G], [sx * (L + Dp), G], [sx * (L + Dp), -G], [sx * L, -G]]); cx.fill();
        cx.strokeStyle = COL[team]; cx.lineWidth = 5 * dpr(); poly([[sx * L, G], [sx * L, -G]], false); cx.stroke();
      }
    } else {
      const key = [W, H, F.length, F.width, F.goal_width, F.goal_depth].join();
      if (key !== pitchKey) {
        pitchKey = key; pitchCache = pitchCache || document.createElement("canvas"); pitchCache.width = W; pitchCache.height = H;
        const keep = cx; cx = pitchCache.getContext("2d"); drawPitch(W, H); cx = keep;
      }
      cx.drawImage(pitchCache, 0, 0);
    }
    const f = S.play.flow;
    if (f && f.phase === "kickoff") {        // kickoff spots, breathing
      const fr = S.params.kickoff_x_frac || 0.27;
      for (const rid of f.playing) {
        const team = teamOf(rid), x = (team === "blue" ? -1 : 1) * fr * F.length;
        cx.save(); cx.setLineDash([6 * dpr(), 5 * dpr()]); cx.lineDashOffset = CALM ? 0 : -now / 60; cx.strokeStyle = SOFT[team]; cx.lineWidth = 2.5 * dpr();
        cx.globalAlpha = .6 + .4 * Math.sin(now / 300); poly(circlePts(x, 0, F.robot_radius)); cx.stroke(); cx.restore();
      }
    }
  }

  // ---- objects ----
  function drawToken(p, team, alpha, hollow, tag) {     // kit-coloured token with a nose where it faces
    const r = S.field.robot_radius * 0.86, yaw = p.yaw || 0, c = Math.cos(yaw), s = Math.sin(yaw);
    const T = (u, v) => [p.x + c * u - s * v, p.y + s * u + c * v];
    const [qx, qy] = M.p(p.x, p.y, tag), R = px(r);
    cx.save(); cx.globalAlpha = alpha;
    if (!hollow) { cx.fillStyle = "#0000004d"; cx.beginPath(); cx.ellipse(qx + R * .12, qy + R * .18, R * 1.02, R * .96, 0, 0, TAU); cx.fill(); }
    const disk = tagged(circlePts(p.x, p.y, r, 40), tag);
    if (hollow) { cx.fillStyle = COL[team] + "40"; poly(disk); cx.fill(); }
    else { const g = cx.createRadialGradient(qx - R * .3, qy - R * .35, R * .1, qx, qy, R * 1.05); g.addColorStop(0, SOFT[team]); g.addColorStop(.55, COL[team]); g.addColorStop(1, DEEP[team]);
           cx.fillStyle = g; poly(disk); cx.fill(); }
    cx.strokeStyle = "#ffffff"; cx.lineWidth = Math.max(2, R * .1); poly(disk); cx.stroke();
    if (hollow) { cx.strokeStyle = COL[team]; cx.lineWidth = Math.max(1.5, R * .06); poly(disk); cx.stroke(); }
    if (p.yaw !== undefined) {
      const nose = tagged([T(r * 1.55, 0), T(r * 1.02, r * .42), T(r * 1.02, -r * .42)], tag);
      cx.fillStyle = hollow ? COL[team] : "#ffffff"; cx.strokeStyle = hollow ? "#fff" : DEEP[team]; cx.lineWidth = Math.max(1, R * .05);
      poly(nose); cx.fill(); cx.stroke();
      const chev = tagged([T(-r * .25, r * .42), T(r * .45, 0), T(-r * .25, -r * .42), T(-r * .02, 0)], tag);
      cx.fillStyle = hollow ? "#ffffffdd" : "#ffffffee"; poly(chev); cx.fill();
    }
    cx.restore();
  }
  function drawSeen(p, team) {   // sim only: where the camera thinks it is
    const r = S.field.robot_radius;
    cx.save(); cx.strokeStyle = SOFT[team]; cx.globalAlpha = .85; cx.lineWidth = Math.max(1.5, px(0.012));
    cx.setLineDash([px(0.035), px(0.025)]); poly(circlePts(p.x, p.y, r, 40)); cx.stroke(); cx.setLineDash([]);
    if (p.yaw !== undefined) { poly([[p.x + Math.cos(p.yaw) * r * .7, p.y + Math.sin(p.yaw) * r * .7], [p.x + Math.cos(p.yaw) * r * 1.2, p.y + Math.sin(p.yaw) * r * 1.2]], false); cx.stroke(); }
    cx.restore();
  }
  function drawLost(p, tag) {
    cx.save(); cx.strokeStyle = "#c7d3df"; cx.globalAlpha = .7; cx.lineWidth = Math.max(2, px(0.014)); cx.setLineDash([px(0.03), px(0.03)]);
    poly(tagged(circlePts(p.x, p.y, S.field.robot_radius, 40), tag)); cx.stroke(); cx.restore();
  }
  // name + role tag above the robot (screen space, upright on the camera image); placed tags are returned so
  // the next one moves up instead of covering it
  function drawLabel(p, rid, team, tag, lost, placed) {
    const pr = S.play.robots[rid] || {}, role = lost ? "NOT SEEN" : pr.playing ? (pr.role === "attack" ? "ATTACK" : pr.role === "defend" ? "DEFEND" : "") : "";
    const [x, y0] = M.p(p.x, p.y, tag), fs = Math.max(13 * dpr(), Math.min(26 * dpr(), px(0.075)));
    cx.save(); cx.font = `700 ${fs}px ${DISPLAY}`; cx.textBaseline = "middle";
    const name = rid.toUpperCase(), pad = fs * .45, w1 = cx.measureText(name).width + 2 * pad, w2 = role ? cx.measureText(role).width + 2 * pad : 0, h = fs * 1.25;
    const x0 = x - (w1 + w2) / 2, sk = h * .22;
    let y = y0 - px(S.field.robot_radius) * 1.15 - fs * .5;
    for (const b of placed) if (x0 < b.x1 && x0 + w1 + w2 > b.x0 && y > b.y0 - h && y - h < b.y1) y = b.y0 - 2;
    placed.push({ x0, x1: x0 + w1 + w2, y0: y - h, y1: y });
    const para = (a, wd, fill) => { cx.beginPath(); cx.moveTo(a + sk, y - h); cx.lineTo(a + wd + sk, y - h); cx.lineTo(a + wd - sk, y); cx.lineTo(a - sk, y); cx.closePath(); cx.fillStyle = fill; cx.fill(); };
    cx.shadowColor = "#0008"; cx.shadowBlur = 8 * dpr(); cx.shadowOffsetY = 2 * dpr();
    para(x0, w1, lost ? "#5f7590" : COL[team]); cx.shadowBlur = 0; cx.shadowOffsetY = 0;
    if (role) para(x0 + w1, w2, lost ? "#2a3a4d" : "#ffffff");
    cx.fillStyle = "#fff"; cx.fillText(name, x0 + pad, y - h / 2 + fs * .08);
    if (role) { cx.fillStyle = lost ? "#c7d3df" : DEEP[team]; cx.fillText(role, x0 + w1 + pad, y - h / 2 + fs * .08); }
    cx.restore();
  }
  function drawBall(b, ghost) {
    const r = S.field.ball_radius, pts = circlePts(b.x, b.y, r, 28), [qx, qy] = M.p(b.x, b.y), R = px(r);
    cx.save();
    if (ghost) { cx.setLineDash([px(0.025), px(0.018)]); cx.strokeStyle = M.cam ? "#fff" : VOLT; cx.lineWidth = Math.max(2, R * .16); poly(pts); cx.stroke();
                 if (M.cam) { cx.setLineDash([]); cx.strokeStyle = VOLT; cx.lineWidth = Math.max(1.5, R * .08); poly(circlePts(b.x, b.y, r * 1.25, 28)); cx.stroke(); } }
    else {
      cx.fillStyle = "#0000005a"; cx.beginPath(); cx.ellipse(qx + R * .25, qy + R * .3, R, R * .92, 0, 0, TAU); cx.fill();
      cx.shadowColor = VOLT; cx.shadowBlur = R * 1.2;
      const g = cx.createRadialGradient(qx - R * .35, qy - R * .4, R * .1, qx, qy, R); g.addColorStop(0, "#fff6c8"); g.addColorStop(.5, VOLT); g.addColorStop(1, "#d39b00");
      cx.fillStyle = g; poly(pts); cx.fill(); cx.shadowBlur = 0;
      cx.strokeStyle = "#5a4100aa"; cx.lineWidth = Math.max(1, R * .07);
      cx.beginPath(); cx.arc(qx, qy, R * .55, -.6, 1.9); cx.stroke();
    }
    cx.restore();
  }
  function drawTarget(tgt, team, now) {   // a slowly turning reticle with a ping
    const [x, y] = M.p(tgt[0], tgt[1]), R = Math.max(9 * dpr(), px(0.07)), col = SOFT[team], rot = CALM ? 0 : now / 1400;
    cx.save(); cx.strokeStyle = col; cx.lineWidth = Math.max(2, R * .14); cx.shadowColor = "#000a"; cx.shadowBlur = 4 * dpr();
    cx.beginPath(); cx.arc(x, y, R, 0, TAU); cx.stroke();
    for (let k = 0; k < 4; k++) { const a = rot + k * Math.PI / 2; cx.beginPath(); cx.moveTo(x + Math.cos(a) * R * 1.25, y + Math.sin(a) * R * 1.25); cx.lineTo(x + Math.cos(a) * R * 1.7, y + Math.sin(a) * R * 1.7); cx.stroke(); }
    cx.fillStyle = col; cx.beginPath(); cx.arc(x, y, R * .22, 0, TAU); cx.fill();
    if (!CALM) { const u = (now % 1400) / 1400; cx.globalAlpha = .6 * (1 - u); cx.lineWidth = Math.max(1.5, R * .08); cx.beginPath(); cx.arc(x, y, R * (1 + 1.4 * u), 0, TAU); cx.stroke(); }
    cx.restore();
  }
  function curve(q) {   // a smooth curve through screen points (midpoint quadratic spline)
    cx.beginPath(); cx.moveTo(...q[0]);
    for (let i = 1; i < q.length - 1; i++) cx.quadraticCurveTo(q[i][0], q[i][1], (q[i][0] + q[i + 1][0]) / 2, (q[i][1] + q[i + 1][1]) / 2);
    cx.lineTo(...q[q.length - 1]);
  }
  function drawPlan(p, team, plan, now) {   // the brain's committed path (debug "plan": [[x, y], ...])
    const pts = plan.filter((q) => Array.isArray(q) && isFinite(q[0]) && isFinite(q[1]));
    if (!pts.length) return;
    const q = [M.p(p.x, p.y, M.cam), ...pts.map((v) => M.p(v[0], v[1]))];
    cx.save(); cx.lineCap = "round"; cx.lineJoin = "round";
    cx.strokeStyle = COL[team] + "55"; cx.lineWidth = Math.max(6, px(0.05)); curve(q); cx.stroke();
    cx.strokeStyle = SOFT[team]; cx.lineWidth = Math.max(2.5, px(0.016));
    cx.setLineDash([px(0.05), px(0.035)]); cx.lineDashOffset = CALM ? 0 : -now / 25; curve(q); cx.stroke();
    cx.restore();
  }
  function drawAim(p, team, tgt, now) {    // no plan: a faint marching line to the target
    cx.save(); cx.strokeStyle = SOFT[team]; cx.globalAlpha = .7; cx.lineWidth = Math.max(1.5, px(0.012)); cx.lineCap = "round";
    cx.setLineDash([px(0.02), px(0.045)]); cx.lineDashOffset = CALM ? 0 : -now / 30;
    cx.beginPath(); cx.moveTo(...M.p(p.x, p.y, M.cam)); cx.lineTo(...M.p(tgt[0], tgt[1])); cx.stroke(); cx.restore();
  }
  function drawTrail(rid, p, team, nowS, tag) {   // a short comet tail behind each robot
    const tr = trails[rid] || (trails[rid] = []);
    if (p && (!tr.length || nowS - tr[tr.length - 1].t > 0.033)) {
      if (tr.length && Math.hypot(p.x - tr[tr.length - 1].x, p.y - tr[tr.length - 1].y) > 0.5) tr.length = 0;
      tr.push({ x: p.x, y: p.y, t: nowS });
    }
    while (tr.length && nowS - tr[0].t > TRAIL_S) tr.shift();
    if (tr.length < 2) return;
    const q = tr.map((v) => M.p(v.x, v.y, tag)), wmax = px(S.field.robot_radius) * 1.1;
    cx.save(); cx.lineCap = "round"; cx.strokeStyle = COL[team];
    for (let i = 1; i < q.length; i++) {
      const u = 1 - (nowS - tr[i].t) / TRAIL_S;
      cx.globalAlpha = .32 * u * u; cx.lineWidth = Math.max(1, wmax * u);
      cx.beginPath(); cx.moveTo(...q[i - 1]); cx.lineTo(...q[i]); cx.stroke();
    }
    cx.restore();
  }
  // predicted ball path: the brain's rolling model (straight line, deceleration ball_decel, kept on the field)
  function ballPath(b, v) {
    const sp = Math.hypot(v.vx || 0, v.vy || 0);
    if (sp < 0.05) return [];
    const a = Math.max(1e-6, +S.params.ball_decel || 0.45), F = S.field, r = F.ball_radius, out = [];
    const lx = F.length / 2 - r, ly = F.width / 2 - r, cl = (q, m) => Math.max(-m, Math.min(m, q)), hEnd = Math.min(PRED_H, sp / a);
    for (let h = PRED_STEP; h <= hEnd + 1e-9; h += PRED_STEP) {
      const s = (sp * h - 0.5 * a * h * h) / sp;
      out.push([cl(b.x + v.vx * s, lx), cl(b.y + v.vy * s, ly), h]);
    }
    out.stopped = sp / a <= PRED_H;
    return out;
  }
  function drawPrediction(b, v, now) {
    const pts = ballPath(b, v);
    if (!pts.length) return;
    const q = [[...M.p(b.x, b.y), 0], ...pts.map(([x, y, h]) => [...M.p(x, y), h])], R = px(S.field.ball_radius);
    cx.save(); cx.lineCap = "round"; cx.strokeStyle = VOLT;
    for (let i = 1; i < q.length; i++) {
      const u = 1 - q[i][2] / (PRED_H + PRED_STEP);
      cx.globalAlpha = .5 * u; cx.lineWidth = Math.max(2, R * (.35 + 1.1 * u));
      cx.beginPath(); cx.moveTo(q[i - 1][0], q[i - 1][1]); cx.lineTo(q[i][0], q[i][1]); cx.stroke();
    }
    if (!CALM && q.length > 2) {
      cx.fillStyle = "#fffbe6";
      for (let k = 0; k < 3; k++) {
        const u = ((now / 1100) + k / 3) % 1, f = u * (q.length - 1), i = Math.floor(f), w = f - i, a = q[i], c = q[Math.min(i + 1, q.length - 1)];
        cx.globalAlpha = .9 * (1 - u); cx.beginPath(); cx.arc(a[0] + (c[0] - a[0]) * w, a[1] + (c[1] - a[1]) * w, Math.max(2, R * .28 * (1 - u * .6)), 0, TAU); cx.fill();
      }
    }
    if (pts.stopped) {
      const e = q[q.length - 1]; cx.globalAlpha = .9; cx.strokeStyle = VOLT; cx.lineWidth = Math.max(2, R * .14); cx.setLineDash([R * .3, R * .22]);
      cx.beginPath(); cx.arc(e[0], e[1], R * .95, 0, TAU); cx.stroke();
    }
    cx.restore();
  }
  function drawGoalFx(now) {   // rings ripple out of the net that was scored into
    if (!goalFx) return;
    const el = (now - goalFx.t0) / 1000; if (el > 2.2) { goalFx = null; return; }
    const F = S.field, sx = goalFx.team === "blue" ? 1 : -1, [x, y] = M.p(sx * F.length / 2, 0);
    cx.save(); cx.strokeStyle = COL[goalFx.team];
    for (let k = 0; k < 3; k++) {
      const u = el / 1.6 - k * .18; if (u <= 0 || u >= 1) continue;
      cx.globalAlpha = .8 * (1 - u); cx.lineWidth = Math.max(3, px(0.05) * (1 - u));
      cx.beginPath(); cx.arc(x, y, px(0.2 + 1.6 * u), 0, TAU); cx.stroke();
    }
    cx.restore();
  }

  // ---- frame ----
  function frame(now) {
    requestAnimationFrame(frame);
    const dt = Math.min(0.1, (now - last) / 1000); last = now;
    S = opts.state(); if (!S) return;
    try { ingest(S); draw(now, dt); } catch (e) { console.error("pitch", e); }   // a drawing bug must not stop the loop
  }
  function draw(now, dt) {
    pollCam();
    const W = Math.round(cv.clientWidth * dpr()), H = Math.round(cv.clientHeight * dpr());
    if (!W || !H) return;
    if (cv.width !== W || cv.height !== H) { cv.width = W; cv.height = H; }
    cx.clearRect(0, 0, W, H);
    M = mapping(W, H);
    drawField(W, H, now);
    if (!M.ok) return;
    const nowS = now / 1000, T = S.truth, Wd = S.world || { robots: {} }, stale = +S.params.stale_stop_s || 0.5;
    const ids = [...new Set([...Object.keys(Wd.robots || {}), ...Object.keys((T && T.robots) || {})])], pose = {};
    for (const rid of ids) {
      const truth = T && T.robots[rid] ? sample("t:" + rid, nowS, dt) : null, seen = Wd.robots[rid] ? sample("w:" + rid, nowS, dt) : null;
      pose[rid] = { truth, seen, lost: !Wd.robots[rid] || Wd.robots[rid].age > stale, anchor: truth || seen, tag: M.cam && !truth };
    }
    for (const rid of Object.keys(trails)) if (!pose[rid]) delete trails[rid];
    for (const rid of ids) {   // under everything: trails, plans, targets
      const p = pose[rid], tm = teamOf(rid), d = S.debug[rid] || {};
      if (!p.anchor) continue;
      drawTrail(rid, p.anchor, tm, nowS, p.tag);
      if (!(S.play.robots[rid] || {}).playing || p.lost) continue;
      if (Array.isArray(d.plan) && d.plan.length) drawPlan(p.anchor, tm, d.plan, now);
      else if (d.target) drawAim(p.anchor, tm, d.target, now);
      if (d.target) drawTarget(d.target, tm, now);
    }
    drawGoalFx(now);
    const bt = T && T.ball ? sample("tb", nowS, dt) : null, bw = Wd.ball ? sample("wb", nowS, dt) : null;
    if (bw && Wd.ball.age < 1.0) drawPrediction(bw, Wd.ball, now);
    if (bt) drawBall(bt, false);
    if (bw) drawBall(bw, !!bt || M.cam);
    const placed = [];
    for (const rid of ids) {
      const p = pose[rid], tm = teamOf(rid), alpha = (S.play.robots[rid] || {}).playing ? 1 : .55;
      if (p.truth) drawToken(p.truth, tm, alpha, false, false);
      if (p.seen) { if (p.lost) drawLost(p.seen, M.cam); else if (p.truth) drawSeen(p.seen, tm); else drawToken(p.seen, tm, alpha, M.cam, M.cam); }
      if (p.anchor) drawLabel(p.anchor, rid, tm, p.tag, p.lost, placed);
    }
  }
  requestAnimationFrame(frame);
  return {
    canvas: cv,
    goal(team) { goalFx = { team, t0: performance.now() }; },
    camOk: () => camOk,
  };
}
