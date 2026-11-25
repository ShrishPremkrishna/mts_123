// Robot card: one robot's state and the operator's actions for it. The same component is used on the home
// screen (setup before play) and in the Play screen's Robots drawer, so both always show the same thing.
//   robotCard(state, robot, ops, { jobs: true })
import { h, NAME, cap } from "/kit/kit.js";

export function robotCard(s, r, ops, opt = {}) {
  const pr = s.play.robots[r.id] || {}, hl = pr.health || { level: "off", text: "" }, f = s.play.flow, job = f && f.roles[r.id];
  const op = s.ops[r.id], running = op && op.running, es = s.estop[r.id];
  const prog = running && op.op === "ready" ? Math.min(100, ((op.progress || {}).elapsed || 0) / 1.2) : null;
  const real = s.source !== "sim" && !!(r.host || s.dry_run_ops);
  const btn = (label, name, args, props = {}, run = {}) =>
    h("button", { class: "btn", type: "button", onclick: () => ops.run(name, args, run), ...props }, label);
  return h("div", { class: "card", "data-team": r.team },
    h("div", { class: "card-head" }, h("span", { class: "display" }, NAME(r.id)), r.host ? h("span", { class: "chip" }, r.host) : null,
      h("i", { class: "light", "data-level": hl.level, title: hl.text })),
    h("div", { class: "st" }, cap(hl.text)),
    opt.jobs ? h("div", { class: "seg", "aria-disabled": String(!job) }, ["auto", "attack", "defend"].map((j) =>
      h("button", { type: "button", "aria-pressed": String(job === j), onclick: () => ops.run("match.job", { robot: r.id, job: j }) },
        j === "defend" ? "Goalie" : cap(j)))) : null,
    prog != null ? h("div", { class: "meter" }, h("i", { style: { width: prog + "%" } })) : null,
    h("div", { class: "acts" },
      real ? btn(running && op.op === "ready" ? "Getting ready…" : es ? "Re-arm" : pr.ready ? "Ready ✓" : "Ready", "robot.ready", { robot: r.id },
        { "data-tone": "go", "aria-disabled": String(!!running || !!pr.ready) }) : null,
      real ? btn("Lie down", "robot.down", { robot: r.id }) : null,
      es && s.source === "sim" ? btn("Clear e-stop", "estop.clear", { robot: r.id }, { "data-tone": "go" }) : null,
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
