// Robot card: one robot's state and the operator's actions for it. The same component is used on the home
// screen (setup before play) and in the Play screen's Robots drawer, so both always show the same thing.
//   robotCard(state, robot, ops, { jobs: true })
import { h, NAME, cap, ask } from "/kit/kit.js";

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const busy = new Set();          // robots whose setup sequence is running in this page

// run fcctl status now and wait for its result: the robot's play state with a fresh setup step, or null
async function freshStatus(ops, rid) {
  const r = await ops.run("robot.status", { robot: rid });
  if (!r || !r.ok) return null;
  for (const t0 = performance.now(); performance.now() - t0 < 70000;) {
    await sleep(150);
    const s = ops.store.get(), o = s && s.ops[rid];
    if (o && o.seq > r.seq) return null;                           // replaced by another op
    if (o && o.seq === r.seq && !o.running) return s.play.robots[rid] || null;
  }
  return null;
}

// The one setup button. The next step is never taken from a cached phase: it re-checks fcctl status first,
// then asks the question for the step that status shows, re-checks again if the answer took long, and only
// then sends. The server refuses robot.ready on an old phase, and homing without home=true (stand confirmed).
export async function setupNext(ops, r) {
  if (busy.has(r.id)) return;
  busy.add(r.id);
  try {
    let pr = await freshStatus(ops, r.id);
    if (!pr) return;
    const su = pr.setup || {};
    if (!su.op || su.op === "robot.status") { ops.toast(`${NAME(r.id)}: ${su.reason || su.label}`, su.op ? "bad" : null); return; }
    const o = ops.get(su.op);
    if (!o) return;
    if (su.ask && !await ask({ title: `${su.label} ${NAME(r.id)}`, lines: su.ask.split(/(?<=[.?:])\s+/), yes: `${su.label} ${NAME(r.id)}`, team: r.team })) return;
    const now = (ops.store.get().play.robots[r.id] || {});
    if (now.phase_age == null || now.phase_age > 10) {             // the answer took a while: check once more
      pr = await freshStatus(ops, r.id);
      if (!pr) return;
      if ((pr.setup || {}).step !== su.step) { ops.toast(`${NAME(r.id)} changed: ${pr.setup.reason || pr.setup.label}. Press again`, "bad"); return; }
    }
    await ops.run(su.op, { robot: r.id, ...(su.args || {}), ...(o.params.confirm ? { confirm: true } : {}) }, { confirmed: true });
  } finally { busy.delete(r.id); }
}

export function robotCard(s, r, ops, opt = {}) {
  const pr = s.play.robots[r.id] || {}, hl = pr.health || { level: "off", text: "" }, f = s.play.flow, job = f && f.roles[r.id];
  const su = pr.setup || { step: "sim", label: "", op: null, done: 3 }, op = s.ops[r.id], es = s.estop[r.id];
  const prog = su.busy && op && op.op === "ready" ? Math.min(100, ((op.progress || {}).elapsed || 0) / 1.2) : null;
  const real = su.step !== "sim";
  const btn = (label, name, args, props = {}, run = {}) =>
    h("button", { class: "btn", type: "button", onclick: () => ops.run(name, args, run), ...props }, label);
  return h("div", { class: "card", "data-team": r.team },
    h("div", { class: "card-head" }, h("span", { class: "display" }, NAME(r.id)), r.host ? h("span", { class: "chip" }, r.host) : null,
      h("i", { class: "light", "data-level": hl.level, title: hl.text })),
    real ? h("div", { class: "steps", title: "homed, standing, linked" },
      [1, 2, 3].map((k) => h("i", { "data-done": su.done >= k || null, "data-now": su.busy && su.done === k - 1 || null }))) : null,
    h("div", { class: "st" }, cap(real && su.step !== "ready" ? su.reason || hl.text : hl.text)),
    opt.jobs ? h("div", { class: "seg", "aria-disabled": String(!job) }, ["auto", "attack", "defend"].map((j) =>
      h("button", { type: "button", "aria-pressed": String(job === j), onclick: () => ops.run("match.job", { robot: r.id, job: j }) },
        j === "defend" ? "Goalie" : cap(j)))) : null,
    prog != null ? h("div", { class: "meter" }, h("i", { style: { width: prog + "%" } })) : null,
    h("div", { class: "acts" },
      // the one setup button: the backend says what the next step is (play_state robots[id].setup)
      real ? (su.op ? h("button", { class: "btn", type: "button", "data-tone": "go", "aria-disabled": String(busy.has(r.id)),
                                    title: "Checks the robot first, then the next step", onclick: () => setupNext(ops, r) }, busy.has(r.id) ? "Checking…" : su.label)
                    : h("button", { class: "btn", type: "button", "data-tone": su.step === "ready" ? "go" : null, "aria-disabled": "true" },
                        su.step === "ready" ? "Ready ✓" : su.label)) : null,
      real && su.done >= 1 && !su.busy ? btn("Lie down", "robot.down", { robot: r.id }) : null,
      es && !real ? btn("Clear e-stop", "estop.clear", { robot: r.id }, { "data-tone": "go" }) : null,
      btn(`E-stop ${NAME(r.id)}`, "estop", { robot: r.id }, { "data-tone": "stop" }, { done: `E-stop sent to ${NAME(r.id)}`, level: "bad" })));
}

// the field camera as a card: found, calibrated, frame rate
export function cameraCard(s) {
  const e = s.eye, ok = e && e.calibrated, field = ok && /field [\d.]+ x [\d.]+ m/.exec(e.message || "");
  return h("div", { class: "card" },
    h("div", { class: "card-head" }, h("span", { class: "display" }, "FIELD CAM"), h("i", { class: "light", "data-level": ok ? (e.paused ? "bad" : "ok") : e ? "warn" : "off" })),
    h("div", { class: "steps" }, h("i", { "data-done": e || null }), h("i", { "data-done": ok || null }), h("i", { "data-done": ok && e.fps >= 15 || null })),
    h("div", { class: "st" }, !e ? "Starts with Play" : e.paused ? "Picture frozen: check the phone" : ok ? `${field ? field[0] + ", " : ""}${Math.round(e.fps)} fps` : e.message));
}
