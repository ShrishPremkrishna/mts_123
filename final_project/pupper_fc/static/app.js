/* Same-origin, local simulator only. Coordinates are metres; yaw is CCW from +x. */
"use strict";
const $ = id => document.getElementById(id);
const canvas = $("pitch"), ctx = canvas.getContext("2d");
let live = null, shown = null, connected = false, polling = false, scenarios = [];
let replay = null, replayIndex = 0, replayPlaying = false, replayClock = 0, replayTick = 0;
let drag = null, dragSending = false, dragPending = null, dragTimer = null;
let geometry = {scale: 1, cx: 0, cy: 0}, eventKey = "";
const names = {delay: "delay_ms", noise: "noise_cm", "max-speed": "max_speed"};
const number = (value, digits = 1) => Number(value || 0).toFixed(digits);
const phaseName = value => String(value || "waiting").replaceAll("_", " ").toLowerCase().replace(/^\w/, c => c.toUpperCase());
function notice(message, retry = false) {
  $("notice-text").textContent = message;
  $("notice").hidden = !message;
  $("reconnect").hidden = !retry;
  $("dismiss").hidden = retry;
}
async function request(path, body) {
  const response = await fetch(path, {method: body ? "POST" : "GET", cache: "no-store",
    headers: body ? {"Content-Type": "application/json"} : {}, body: body ? JSON.stringify(body) : undefined,
    signal: AbortSignal.timeout(4000)});
  if (!response.ok) {
    const error = await response.json().catch(() => ({}));
    const failure = new Error(typeof error.detail === "string" ? error.detail : `Request failed (${response.status}).`);
    failure.serverResponse = true;
    throw failure;
  }
  return response.json();
}
function disconnected() {
  connected = false;
  notice("Local simulator unavailable. Restart the local server, then reconnect.", true);
  updateControls();
  if (!replay) { $("run-status").textContent = "Disconnected"; $("run-status").className = "status stopped"; }
}
async function poll() {
  if (polling) return;
  polling = true;
  try {
    live = await request("/api/state"); connected = true;
    if (!replay) render(live); else updateControls();
  } catch (_) { disconnected(); }
  polling = false;
  if (connected) setTimeout(poll, 100);
}
async function control(action, fields = {}) {
  if (!connected || (replay && action !== "stop")) return;
  try {
    live = await request("/api/control", {action, ...fields});
    if (!replay) render(live); else updateControls();
  } catch (error) {
    if (error.serverResponse) notice(error.message); else disconnected();
  }
}
function updateControls() {
  document.querySelectorAll("[data-live]").forEach(element => {
    element.disabled = !connected || (!!replay && element.id !== "stop");
  });
  if (!replay && connected && live) {
    $("start").disabled = !!live.running;
    $("pause").disabled = !live.running;
    $("step").disabled = !!live.running || !!live.stopped;
    $("resume").disabled = live.execution ? !live.execution.resume_available : live.decision?.phase !== "await_reset" || !!live.stopped;
  }
  if (replay) $("replay").querySelector("span").textContent = live?.running
    ? "Live practice is still running. STOP remains available." : "Recorded frames only. STOP remains available for live practice.";
}
function render(state) {
  shown = state;
  const decision = state.decision || {}, command = decision.command || {}, metrics = state.metrics || {}, execution = state.execution;
  $("pitch-empty").hidden = true;
  $("scenario-title").textContent = scenarios.find(s => s.id === state.scenario)?.name || phaseName(state.scenario || "Practice field");
  const label = replay ? "Recorded playback" : state.stopped ? "Stopped" : state.running ? "Practicing" : "Paused";
  $("run-status").replaceChildren(document.createElement("i"), document.createTextNode(label));
  $("run-status").className = `status ${!replay && state.running ? "running" : state.stopped ? "stopped" : ""}`;
  $("phase").textContent = phaseName(decision.phase);
  $("reason").textContent = decision.reason || "Ready for the next step.";
  $("role").textContent = decision.role ? phaseName(decision.role) : execution ? "Unassigned" : "Not recorded";
  $("role").className = decision.role === "defense" ? "defense" : "";
  $("authority").textContent = !execution ? "Not recorded" : execution.armed ? "Armed" : "Stopped";
  $("authority").className = execution && !execution.armed ? "stopped" : "";
  $("lease").textContent = Number.isFinite(execution?.lease_remaining_s) ? `${number(Math.max(0, execution.lease_remaining_s) * 1000, 0)} ms` : "—";
  $("epoch").textContent = execution?.epoch || "—";
  $("recovery").hidden = !decision.recovery_requested;
  $("tracking-age-label").textContent = execution ? "Decision input age" : "Recorded observation age";
  const capture = execution ? execution.frame_captured_at : state.observation?.captured_at;
  const age = Number.isFinite(capture) ? Math.max(0, state.time - capture) : null;
  $("tracking-age").textContent = age === null ? "Unavailable" : `${number(age * 1000, 0)} ms`;
  $("speed").textContent = `${number(Math.hypot(command.vx || 0, command.vy || 0), 2)} m/s`;
  $("turn").textContent = `${number(command.wz, 2)} rad/s`;
  $("time").textContent = `${number(state.time, 2)} s`;
  for (const [id, key] of [["goals", "goals"], ["shots", "shots"], ["own-goals", "own_goals"], ["phase-changes", "phase_changes"]]) $(id).textContent = metrics[key] || 0;
  $("goal-prompt").hidden = !(state.truth?.goal || String(decision.phase).toLowerCase().includes("goal"));
  let goalMessage = "Goal recorded. Return home is pending.";
  if (state.truth?.fallen || decision.phase === "recovery_required") goalMessage = "Goal recorded. Return home is pending while Pupper is fallen.";
  else if (state.stopped || decision.phase === "stopped") goalMessage = "Goal recorded. Practice is stopped; return home is pending.";
  else if (decision.phase === "await_reset") goalMessage = "Ready for the next play. Resume places the ball at center.";
  else if (!state.running) goalMessage = "Goal recorded. Practice is paused; return home is pending.";
  else if (decision.phase === "return_home") goalMessage = "Goal recorded. Pupper is returning home.";
  if (execution && !execution.resume_available && execution.resume_reason) goalMessage = `Resume unavailable: ${execution.resume_reason}`;
  $("goal-message").textContent = goalMessage;
  const settings = state.settings || {};
  for (const [id, key] of Object.entries(names)) {
    if (document.activeElement !== $(id) && settings[key] !== undefined) $(id).value = settings[key];
    sliderLabel(id);
  }
  $("hide-ball").checked = settings.ball_visible === false;
  $("freeze").checked = !!settings.freeze;
  $("camera-hold").checked = !!execution?.external_hold;
  $("intent-loss").checked = settings.renew_intent === false;
  $("fallen").checked = !!state.truth?.fallen;
  const events = (state.events || []).slice(-6).reverse(), key = JSON.stringify(events);
  if (key !== eventKey) {
    eventKey = key;
    $("events").replaceChildren(...events.map(event => {
      const li = document.createElement("li"), time = document.createElement("time"), phase = document.createElement("b"), reason = document.createElement("span");
      time.textContent = `${number(event.time)}s`; phase.textContent = [event.role && phaseName(event.role), phaseName(event.phase)].filter(Boolean).join(" · "); reason.textContent = event.reason;
      li.append(time, phase, reason); return li;
    }));
    if (!events.length) { const li = document.createElement("li"); li.className = "empty-event"; li.textContent = "Start practice to follow the decisions."; $("events").append(li); }
  }
  updateControls(); draw();
}
function sliderLabel(id) {
  $(id + "-value").textContent = id === "delay" ? `${$(id).value} ms` : id === "noise" ? `${$(id).value} cm` : `${number($(id).value, 2)} m/s`;
}
function position(x, y) { return [geometry.cx + x * geometry.scale, geometry.cy - y * geometry.scale]; }
function line(x1, y1, x2, y2, color, width = 1.2, dash = []) {
  const a = position(x1, y1), b = position(x2, y2); ctx.beginPath(); ctx.setLineDash(dash); ctx.moveTo(...a); ctx.lineTo(...b); ctx.strokeStyle = color; ctx.lineWidth = width; ctx.stroke(); ctx.setLineDash([]);
}
function circle(x, y, radius, color, fill = false, width = 1.2) {
  ctx.beginPath(); ctx.arc(...position(x, y), radius * geometry.scale, 0, Math.PI * 2); ctx.lineWidth = width; ctx.strokeStyle = color; ctx.fillStyle = color; fill ? ctx.fill() : ctx.stroke();
}
function rect(x, y, w, h, color, fill = false) {
  const p = position(x, y); ctx.fillStyle = color; ctx.strokeStyle = color; ctx.lineWidth = 1.2;
  fill ? ctx.fillRect(p[0], p[1], w * geometry.scale, -h * geometry.scale) : ctx.strokeRect(p[0], p[1], w * geometry.scale, -h * geometry.scale);
}
function label(text, x, y, color = "#a7c4ac", size = 9, align = "center") {
  ctx.fillStyle = color; ctx.font = `500 ${size}px system-ui, sans-serif`; ctx.textAlign = align; ctx.fillText(text, ...position(x, y));
}
function robot(pose, ghost, fallen, field) {
  if (!pose) return;
  const s = geometry.scale; ctx.save(); ctx.translate(...position(pose.x, pose.y)); ctx.rotate(-pose.yaw);
  ctx.globalAlpha = ghost ? .35 : 1;
  if (ghost) { ctx.strokeStyle = "#e3f1e4"; ctx.setLineDash([4, 3]); ctx.strokeRect(-.15*s, -.13*s, .3*s, .26*s); }
  else {
    ctx.fillStyle = "#15261e";
    for (const x of [-.11, .10]) for (const y of [-.16, .11]) ctx.fillRect(x*s-.035*s, y*s, .07*s, .05*s);
    ctx.shadowColor = "#0b1c1680"; ctx.shadowBlur = 9; ctx.shadowOffsetY = 3;
    ctx.fillStyle = fallen ? "#d68e75" : "#e2e5d6"; ctx.fillRect(-.15*s, -.10*s, .30*s, .20*s);
    ctx.shadowBlur = 0; ctx.shadowOffsetY = 0; ctx.fillStyle = "#dec06d"; ctx.fillRect(-.06*s, -.10*s, .055*s, .20*s);
    ctx.fillStyle = "#344a3a"; ctx.fillRect(.07*s, -.055*s, .045*s, .11*s);
    ctx.fillStyle = fallen ? "#e5a48d" : "#efcc72"; ctx.fillRect((field.bumper_offset-.015)*s, -.14*s, .03*s, .28*s);
    if (fallen) { ctx.strokeStyle = "#692c23"; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(-.06*s,-.06*s); ctx.lineTo(.06*s,.06*s); ctx.moveTo(-.06*s,.06*s); ctx.lineTo(.06*s,-.06*s); ctx.stroke(); }
  }
  ctx.strokeStyle = ghost ? "#e3f1e4" : "#f5df9a"; ctx.lineWidth = 1.5; ctx.setLineDash([]);
  ctx.beginPath(); ctx.moveTo(.23*s,0); ctx.lineTo(.39*s,0); ctx.lineTo(.34*s,-.035*s); ctx.moveTo(.39*s,0); ctx.lineTo(.34*s,.035*s); ctx.stroke(); ctx.restore();
}
function draw() {
  const bounds = canvas.getBoundingClientRect(), ratio = window.devicePixelRatio || 1;
  canvas.width = Math.round(bounds.width * ratio); canvas.height = Math.round(bounds.height * ratio);
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0); ctx.clearRect(0, 0, bounds.width, bounds.height);
  if (!shown?.truth) return;
  const field = shown.field, truth = shown.truth, observation = shown.observation || {}, decision = shown.decision || {};
  const l = field.length / 2, w = field.width / 2, g = field.goal_width / 2, sign = field.attack_sign || 1;
  geometry = {scale: Math.min((bounds.width - 58) / (field.length + .32), (bounds.height - 45) / (field.width + .22)), cx: bounds.width / 2, cy: bounds.height / 2};
  rect(-l, -w, field.length, field.width, "#315d40", true);
  for (let i = 0; i < 8; i += 2) rect(-l + field.length * i/8, -w, field.length / 8, field.width, "#355f43", true);
  rect(-l, -w, field.length, field.width, "#b6d1b78c"); line(0,-w,0,w,"#b6d1b778"); circle(0,0,.36,"#b6d1b778"); circle(0,0,.018,"#b6d1b7",true);
  for (const end of [-1, 1]) {
    const x = end * l; rect(end < 0 ? x : x - .4, -.64, .4, 1.28, "#b6d1b770");
    rect(end < 0 ? x-.15 : x, -g, .15, field.goal_width, "#b6d1b714", true);
    for (let i=1; i<6; i++) line(x, -g+i*field.goal_width/6, x+end*.15, -g+i*field.goal_width/6, "#b6d1b730", .7);
    rect(end < 0 ? x-.15 : x, -g, .15, field.goal_width, "#b6d1b780");
    line(x,-g,x,g, end === sign ? "#d5e9ad" : "#a2c9c4", 3);
    label(end === sign ? "ATTACK" : "HOME", x-end*.2, w+.12, end === sign ? "#c8ddb4" : "#9dbab3", 8);
  }
  label("+x →", l, -w-.15, "#849b8b", 8, "right"); label("+y ↑", -l, -w-.15, "#849b8b", 8, "left");
  if (decision.target) {
    const [x,y] = decision.target; line(truth.robot.x, truth.robot.y, x, y, "#c7edb875", 1.3, [4,5]);
    circle(x,y,.08,"#c7edb8",false,1.5); line(x-.035,y,x+.035,y,"#c7edb8"); line(x,y-.035,x,y+.035,"#c7edb8");
    if (decision.target_heading != null) line(x,y,x+.25*Math.cos(decision.target_heading),y+.25*Math.sin(decision.target_heading),"#c7edb899",1.5);
  }
  if (observation.robot && (shown.settings?.noise_cm || shown.settings?.delay_ms || shown.settings?.freeze)) robot(observation.robot, true, false, field);
  if (observation.ball && (shown.settings?.noise_cm || shown.settings?.delay_ms || shown.settings?.freeze)) {
    ctx.setLineDash([3,3]); circle(observation.ball.x, observation.ball.y, field.ball_radius+.04, "#e8f0df99"); ctx.setLineDash([]);
  }
  const actualRobot = drag?.kind === "robot" ? {...truth.robot, ...drag.point} : truth.robot;
  const actualBall = drag?.kind === "ball" ? {...truth.ball, ...drag.point} : truth.ball;
  robot(actualRobot, false, truth.fallen, field);
  if (actualBall) {
    ctx.save(); ctx.shadowColor = "#10291888"; ctx.shadowBlur = 6; ctx.shadowOffsetY = 3;
    circle(actualBall.x,actualBall.y,field.ball_radius,"#ffab7b",true); ctx.restore();
    circle(actualBall.x-.013,actualBall.y+.016,.018,"#fff0d1",true);
  }
  if (!observation.ball) label("BALL NOT OBSERVED", 0, -w-.15, "#d5bb93", 8);
}
function fieldPoint(event) {
  const bounds = canvas.getBoundingClientRect(), f = shown.field;
  return {x: Math.max(-f.length/2, Math.min(f.length/2, (event.clientX-bounds.left-geometry.cx)/geometry.scale)), y: Math.max(-f.width/2, Math.min(f.width/2, -(event.clientY-bounds.top-geometry.cy)/geometry.scale))};
}
async function flushDrag() {
  clearTimeout(dragTimer); dragTimer = null;
  if (dragSending || !dragPending) return;
  const pending = dragPending; dragPending = null; dragSending = true;
  await control(`move_${pending.kind}`, pending.point); dragSending = false;
  if (dragPending) dragTimer = setTimeout(flushDrag, 80);
}
canvas.addEventListener("pointerdown", event => {
  if (!connected || replay || !shown || event.button !== 0) return;
  const point = fieldPoint(event), truth = shown.truth;
  const ballDistance = Math.hypot(point.x-truth.ball.x,point.y-truth.ball.y), robotDistance = Math.hypot(point.x-truth.robot.x,point.y-truth.robot.y);
  const kind = ballDistance < .18 && ballDistance <= robotDistance ? "ball" : robotDistance < .26 ? "robot" : null;
  if (!kind) return;
  drag = {kind, point}; canvas.setPointerCapture(event.pointerId); canvas.style.cursor = "grabbing"; draw();
});
canvas.addEventListener("pointermove", event => {
  if (!drag) return;
  drag.point = fieldPoint(event); dragPending = {...drag}; draw();
  if (!dragTimer && !dragSending) dragTimer = setTimeout(flushDrag, 80);
});
function endDrag(event) {
  if (!drag) return;
  dragPending = {...drag}; drag = null; canvas.style.cursor = "";
  if (canvas.hasPointerCapture(event.pointerId)) canvas.releasePointerCapture(event.pointerId);
  flushDrag();
}
canvas.addEventListener("pointerup", endDrag); canvas.addEventListener("pointercancel", endDrag);
new ResizeObserver(draw).observe(canvas);
for (const action of ["start","pause","step","stop","resume"]) $(action).onclick = () => control(action);
$("reset").onclick = () => {
  const seed = Number($("seed").value);
  if (!Number.isInteger(seed) || seed < 0 || seed > 2147483647) return notice("Choose a whole-number seed between 0 and 2147483647.");
  control("reset", {scenario: $("scenario").value, seed, attack_sign: Number($("attack-direction").value)});
};
$("scenario").onchange = () => { $("scenario-description").textContent = scenarios.find(s => s.id === $("scenario").value)?.description || ""; };
for (const [id,key] of Object.entries(names)) {
  $(id).oninput = () => sliderLabel(id);
  $(id).onchange = () => control("configure", {settings: {[key]: Number($(id).value)}});
}
$("hide-ball").onchange = () => control("configure", {settings: {ball_visible: !$("hide-ball").checked}});
$("freeze").onchange = () => control("configure", {settings: {freeze: $("freeze").checked}});
$("camera-hold").onchange = () => control("external_hold", {held: $("camera-hold").checked});
$("intent-loss").onchange = () => control("configure", {settings: {renew_intent: !$("intent-loss").checked}});
$("fallen").onchange = () => control("fall", {value: $("fallen").checked});
$("nudge-up").onclick = () => control("nudge_ball", {vx: 0, vy: .65});
$("nudge-down").onclick = () => control("nudge_ball", {vx: 0, vy: -.65});
$("center-ball").onclick = () => control("move_ball", {x: 0, y: 0});
$("threat-ball").onclick = () => control("move_ball", {x: -.33 * live.field.length * (live.field.attack_sign || 1), y: .22 * live.field.width});
$("home-robot").onclick = () => control("move_robot", {x: -.9*(live?.field.attack_sign || 1), y: 0, yaw: live?.field.attack_sign === -1 ? Math.PI : 0});
document.addEventListener("keydown", event => {
  if (event.code !== "Space" || event.repeat || /^(INPUT|SELECT|TEXTAREA|BUTTON)$/.test(event.target.tagName) || event.target.isContentEditable) return;
  event.preventDefault(); control("stop");
});
$("dismiss").onclick = () => notice("");
$("reconnect").onclick = () => { notice(""); connect(); };
$("export").onclick = async () => {
  try {
    const trace = await request("/api/trace"), url = URL.createObjectURL(new Blob([JSON.stringify(trace)], {type: "application/json"}));
    const link = document.createElement("a"); link.href = url; link.download = `pupper-fc-${live?.scenario || "practice"}-${live?.seed || 0}.json`; link.click(); setTimeout(() => URL.revokeObjectURL(url), 1000);
  } catch (error) { notice(`Could not export trace: ${error.message}`); }
};
$("import").onclick = () => $("trace-file").click();
function validFrame(frame) {
  const f = frame?.field, t = frame?.truth, d = frame?.decision;
  const point = p => p && Number.isFinite(p.x) && Number.isFinite(p.y);
  if (!Number.isFinite(frame?.time) || !f || !t || !d || !point(t.robot) || !Number.isFinite(t.robot.yaw) || !point(t.ball)) return false;
  if (!["length", "width", "goal_width", "ball_radius", "bumper_offset"].every(key => Number.isFinite(f[key]) && f[key] > 0)) return false;
  if (d.target != null && (!Array.isArray(d.target) || d.target.length !== 2 || !d.target.every(Number.isFinite))) return false;
  if (d.target_heading != null && !Number.isFinite(d.target_heading)) return false;
  const observation = frame.observation;
  return (!observation?.robot || (point(observation.robot) && Number.isFinite(observation.robot.yaw))) && (!observation?.ball || point(observation.ball));
}
$("trace-file").onchange = async event => {
  const file = event.target.files[0]; event.target.value = ""; if (!file) return;
  try {
    if (file.size > 30 * 1024 * 1024) throw new Error("Choose a trace smaller than 30 MB.");
    const trace = JSON.parse(await file.text());
    if (trace.schema !== "pupper_fc_trace_v1" || !Array.isArray(trace.frames) || !trace.frames.length || !trace.frames.every(validFrame)) throw new Error("This is not a valid PupperFC trace.");
    let priorState = null; const events = [];
    replay = trace.frames.map(frame => {
      const decision = frame.decision || {}, stateKey = JSON.stringify([decision.phase, decision.role, decision.reason]);
      if (stateKey !== priorState) { events.push({time: frame.time, phase: decision.phase, role: decision.role, reason: decision.reason || ""}); priorState = stateKey; }
      return {...frame, events: events.slice(-6)};
    });
    replayIndex = 0; replayPlaying = false; drag = null; dragPending = null;
    $("replay").hidden = false; $("replay-position").max = replay.length - 1; $("replay-play").textContent = "Play recording"; notice(""); showFrame();
  } catch (error) { notice(`Could not open trace: ${error.message}`); }
};
function showFrame() {
  $("replay-position").value = replayIndex; $("replay-frame").textContent = `${replayIndex+1} / ${replay.length}`;
  render(replay[replayIndex]);
}
$("replay-position").oninput = () => { replayPlaying = false; $("replay-play").textContent = "Play recording"; replayIndex = Number($("replay-position").value); showFrame(); };
$("replay-exit").onclick = () => { replay = null; replayPlaying = false; $("replay").hidden = true; if (live) render(live); };
$("replay-play").onclick = () => {
  replayPlaying = !replayPlaying;
  if (replayIndex >= replay.length-1) replayIndex = 0;
  replayClock = replay[replayIndex].time; replayTick = performance.now();
  $("replay-play").textContent = replayPlaying ? "Pause recording" : "Play recording";
};
setInterval(() => {
  if (!replay || !replayPlaying) return;
  const now = performance.now(); replayClock += (now-replayTick)/1000; replayTick = now;
  while (replayIndex < replay.length-1 && replay[replayIndex+1].time <= replayClock) replayIndex++;
  showFrame();
  if (replayIndex === replay.length-1) { replayPlaying = false; $("replay-play").textContent = "Play recording"; }
}, 50);
async function connect() {
  updateControls();
  try {
    scenarios = await request("/api/scenarios");
    $("scenario").replaceChildren(...scenarios.map(s => { const option = document.createElement("option"); option.value = s.id; option.textContent = s.name; return option; }));
    live = await request("/api/state"); connected = true;
    $("scenario").value = live.scenario; $("seed").value = live.seed; $("attack-direction").value = live.field.attack_sign; $("scenario").onchange();
    if (!replay) render(live); poll();
  } catch (_) { disconnected(); }
}
connect();
