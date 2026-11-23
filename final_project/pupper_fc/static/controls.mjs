import {KEY_GUIDE, readDrive, slotForKey} from './drive.mjs';
import {kickPresentation} from './kick_view.mjs';

const $ = id => document.getElementById(id);
const owner = crypto.randomUUID();
const teams = {}, down = new Set();
const NAMES = {blue: 'Blue', red: 'Red'}, TRACK = {blue: 'striker', red: 'keeper'};
const SLOT_OF = {'pupper-1': 'blue', 'pupper-2': 'red'};
const VIEWS = ['home', 'field', 'robot', 'settings', 'play'];
let state = {phase: 'idle'}, appInfo = null, pending = false, lastError = '', lastDriveEpoch = null;
let survey = null, frozen = null, pickingFar = false, cameraFormLoaded = false;
const frameKeys = {view: '', 'play-view': ''};

const replay = () => (appInfo?.mode || state.mode) === 'replay';
const message = text => { lastError = text; $('error').textContent = text; };
const fmt = (value, digits = 2) => Number.isFinite(value) ? value.toFixed(digits) : '–';
const deg = rad => Number.isFinite(rad) ? Math.round(rad * 180 / Math.PI) + '°' : '–';
function setText(el, text) { if (el.textContent !== text) el.textContent = text; }

/* ---------- views ---------- */
function currentView() { return document.querySelector('main').dataset.view; }
function setView(view) {
  if (!VIEWS.includes(view)) view = 'home';
  document.querySelector('main').dataset.view = view;
  for (const el of document.querySelectorAll('[data-views]')) el.classList.toggle('off-view', !el.dataset.views.split(' ').includes(view));
  for (const button of document.querySelectorAll('.tabs [data-view-target]')) {
    if (button.dataset.viewTarget === view) button.setAttribute('aria-current', 'page');
    else button.removeAttribute('aria-current');
  }
  if (location.hash !== '#' + view) history.replaceState(null, '', '#' + view);
  frameKeys.view = frameKeys['play-view'] = '';
  render();
}
document.addEventListener('click', e => {
  const target = e.target.closest('[data-view-target]');
  if (target) setView(target.dataset.viewTarget);
});
window.addEventListener('hashchange', () => setView(location.hash.slice(1)));

async function post(path, data = {}) {
  const response = await fetch('/api/' + path, {method: 'POST', headers: {'content-type': 'application/json', 'X-Pupper-FC': owner}, body: JSON.stringify(data)});
  const result = await response.json();
  if (!response.ok) throw new Error(result.detail || 'Request failed');
  return result;
}
async function action(path, data) {
  pending = true; message(''); render();
  try { state = await post(path, data); }
  catch (e) { message(e.message); }
  finally { pending = false; render(); }
}

/* ---------- mode and app identity ---------- */
function renderApp() {
  const mode = appInfo?.mode || state.mode;
  if (!mode) return;
  document.body.dataset.mode = mode;
  setText($('mode-badge'), mode === 'replay' ? 'Replay · simulated robots' : 'Live · real robots');
  $('replay-ribbon').hidden = mode !== 'replay';
  setText($('feed-tag'), mode === 'replay' ? 'Rendered simulation' : '');
  setText($('app-mode-label'), mode === 'replay' ? 'Replay mode' : 'Live mode');
  if (!appInfo) return;
  const src = appInfo.source || {};
  setText($('app-limitation'), appInfo.limitation || '');
  setText($('app-source'), src.commit ? src.kind + ' ' + src.commit.slice(0, 12) + (src.dirty ? ' with local changes' : '') : (src.detail || 'Unknown'));
  setText($('app-hardware'), appInfo.hardware_qualified === true ? 'Qualified' : 'Unqualified');
  const build = appInfo.selection?.qualification?.target_build || 'pending';
  setText($('app-native'), build === 'baseline_driver_passed_full_pair_pending'
    ? 'Baseline driver build passed; full robot build pending' : build.replaceAll('_', ' ') + '; paired release pending');
  setText($('app-executor'), appInfo.executor ? 'Selected executor, synthetic robot I/O' : 'Runs on each robot');
  setText($('app-onboard'), appInfo.onboard_feed?.detail || 'No stream contract');
  const limits = appInfo.selection?.limits || [];
  if ($('app-limits').childElementCount !== limits.length) {
    $('app-limits').replaceChildren(...limits.map(text => Object.assign(document.createElement('li'), {textContent: text})));
  }
  if (mode === 'replay') {
    for (const option of $('camera-source').options) option.disabled = option.value !== 'sim';
    $('camera-source').value = 'sim';
  }
}

/* ---------- camera form (Field) ---------- */
const anchorIds = {length: 'anchor-length', width: 'anchor-width', goal_width: 'anchor-mouth',
  tag_size: 'anchor-size', rotation: 'anchor-rotation', inset_x: 'anchor-x', inset_y: 'anchor-y', tag_id: 'anchor-id'};
function cameraForm() {
  const corner = $('field-mode').value === 'corner', colors = $('robot-tracking').value === 'colors';
  $('anchor-fields').hidden = !corner;
  $('patch-height-field').hidden = !colors;
  $('tracking-help').textContent = colors ? 'Centre a matte 9 cm blue or red patch on the back. Place a 3.6 cm bright lime patch 9 cm forward on the same plane. A missing or ambiguous nose holds play.' :
    'Blue wears tag 10 and Red wears tag 11. Point the printed top toward the nose.';
}
function loadCameraForm() {
  if (cameraFormLoaded || !state.field_defaults) return;
  cameraFormLoaded = true;
  const settings = state.camera_settings || {}, f = state.field_defaults;
  $('field-mode').value = settings.field_mode || 'layout';
  $('robot-tracking').value = settings.robot_tracking || 'tags';
  $('patch-height').value = settings.patch_height ?? .18;
  const values = settings.anchor || {length: f.length, width: f.width, goal_width: f.goal_width, tag_size: state.floor_size};
  for (const [key, id] of Object.entries(anchorIds)) if (values[key] != null) $(id).value = values[key];
  $('anchor-corner').value = values.corner || 'west_north';
  cameraForm();
}
$('field-mode').onchange = $('robot-tracking').onchange = cameraForm;

/* ---------- robots ---------- */
function addressKey(slot) { return 'pupper-fc-address-' + slot; }
function savedAddress(slot) { try { return localStorage.getItem(addressKey(slot)) || ''; } catch { return ''; } }
const discoverTeam = slot => post('discover', replay() ? {slot} : {slot, host: savedAddress(slot)});
function clearKeys(slot) { for (const key of down) if (slotForKey(key) === slot) down.delete(key); }

function freshOperatorOffer(offer, limit = 750) {
  if (!offer) return false;
  const elapsed = performance.now() - offer.received, wall = Date.now() - offer.wall;
  return elapsed >= 0 && elapsed < limit && wall >= 0 && wall < limit;
}

function teamStatus(t) {
  if (t.operator?.lost) return ['Stop link lost', 'bad'];
  if (t.stopping) return ['Stopped', 'bad'];
  if (t.online) return t.ready ? [replay() ? 'Simulated · ready' : 'Ready', 'good'] : ['Connected', 'warn'];
  if (t.connecting) return [t.preparing ? 'Preparing…' : t.socket ? 'Connecting…' : 'Searching…', 'warn'];
  if (t.setup.busy) return [t.setup.cue ? 'Ready to stand' : t.setup.action === 'home' ? 'Homing…' : 'Preparing…', 'warn'];
  if (t.operator) {
    if (t.operator.standing) return ['Pi reports standing', 'good'];
    if (t.operator.waiting) return ['Standing requested…', 'warn'];
    return [t.operator.ready ? 'Ready to stand' : 'Linking laptop Stop…', 'warn'];
  }
  if (t.homed) return ['Homed', 'warn'];
  if (t.needsHomeShown) return ['Needs Home', 'warn'];
  if (t.setup.prepared) return ['Prepared', 'warn'];
  return ['Offline', ''];
}

function addTeam(slot, config) {
  const clone = id => $(id).content.firstElementChild.cloneNode(true);
  const pick = (root, names) => Object.fromEntries(names.map(n => [n.replace(/-./g, m => m[1].toUpperCase()), root.querySelector('.' + n)]));
  const t = {slot, id: config.robot_id, name: NAMES[slot], online: false, connecting: false, ready: false, mode: null,
    socket: null, stopping: false, setup: {message: ''}, operator: null, feedback: '', ui: {}};
  teams[slot] = t;

  const kit = clone('kit');
  kit.classList.add(slot);
  kit.querySelector('h2').textContent = t.name + ' Pupper';
  kit.querySelector('.number').textContent = slot === 'blue' ? '1' : '2';
  $('teams').append(kit);
  t.ui.kit = {root: kit, ...pick(kit, ['kit-status', 'kit-note', 'kit-connect'])};
  t.ui.kit.kitConnect.onclick = () => primaryAction(t);

  const life = clone('lifecycle');
  life.classList.add(slot);
  life.querySelector('h3').textContent = t.name + ' Pupper';
  $('lifecycles').append(life);
  t.ui.life = {root: life, ...pick(life, ['life-status', 'address-input', 'save-address', 'connect', 'prepare', 'disconnect', 'supported',
    'home-mode-select', 'home', 'stand', 'stand-hint', 'link-state', 'play-state', 'life-message', 's-connect', 's-home', 's-stand', 's-play'])};
  const L = t.ui.life;
  L.addressInput.value = savedAddress(slot);
  L.connect.onclick = () => t.openRunner();
  L.prepare.onclick = () => beginSetup(t, 'prepare');
  L.disconnect.onclick = () => disconnect(t);
  L.supported.onchange = render;
  L.homeModeSelect.onchange = () => { if (L.homeModeSelect.value === 'stand') openOperator(t); render(); };
  L.home.onclick = () => beginSetup(t, 'home');
  L.stand.onclick = () => standClick(t);
  L.saveAddress.onclick = () => saveAddress(t);

  const ts = clone('team-setting');
  ts.classList.add(slot);
  ts.querySelector('h4').textContent = t.name + ' Pupper';
  $('team-settings').append(ts);
  t.ui.ts = {root: ts, ...pick(ts, ['mode', 'guide', 'ball-mode', 'ball-reading', 'imu-profile', 'feedback'])};
  t.ui.ts.guide.textContent = 'Teleop keys: ' + KEY_GUIDE[slot];
  t.ui.ts.ballMode.onchange = () => { t.requestedBall = t.ui.ts.ballMode.value; t.ui.ts.ballMode.blur(); };
  t.ui.ts.mode.onchange = () => { t.requested = {mode: t.ui.ts.mode.value, expires: performance.now() + 400}; clearKeys(slot); t.ui.ts.mode.blur(); };

  const onboard = clone('onboard');
  onboard.classList.add(slot);
  $('onboards').append(onboard);
  t.ui.onboard = {root: onboard, frame: onboard.querySelector('.onboard-frame span'), caption: onboard.querySelector('figcaption')};
  const tele = clone('tele');
  tele.classList.add(slot);
  tele.querySelector('h3').textContent = t.name;
  $('telemetry').append(tele);
  t.ui.tele = {root: tele, ...pick(tele, ['tele-mode', 'tele-phase', 'tele-reason', 't-pose', 't-target', 't-error', 't-command', 't-age', 't-frames', 't-lease', 't-kick', 't-source'])};

  t.openRunner = (alreadyDiscovered = false) => openRunner(t, alreadyDiscovered);
}

function primaryAction(t) {
  if (t.online || t.setup.prepared || t.homed || t.operator || t.setup.busy || t.needsHomeShown) { setView('robot'); return; }
  void t.openRunner();
}

function disconnect(t) {
  clearKeys(t.slot);
  stopOperator(t);
  t.socket?.close();
}

async function saveAddress(t) {
  if (t.savingAddress || t.setup.busy || t.connecting || t.online || t.operator || state.phase !== 'idle') return;
  t.savingAddress = true; render();
  const host = t.ui.life.addressInput.value.trim();
  try {
    await post('discover', {slot: t.slot, host, save_address: true});
    if (host) localStorage.setItem(addressKey(t.slot), host); else localStorage.removeItem(addressKey(t.slot));
    t.setup = {message: host ? 'Saved SSH address: ' + host : 'Default connection restored.'};
    t.homed = false; t.fcOperator = undefined; t.needsHomeShown = false;
    t.ui.life.supported.checked = false;
  } catch (error) { t.setup.message = error.message; }
  finally { t.savingAddress = false; render(); }
}

/* ---------- Prepare and Home (setup socket) ---------- */
function cancelSetup(t) {
  if (!t.setup.busy) return;
  const socket = t.setup.socket;
  t.setup = {message: 'Setup connection closed. If Home started, keep Pupper on its stand until it finishes.'};
  t.ui.life.supported.checked = false;
  if (socket) socket.close();
}

async function beginSetup(t, step) {
  const L = t.ui.life;
  if (t.online || t.connecting || t.setup.busy || t.operator || state.phase !== 'idle') return;
  const request = {action: step};
  if (step === 'home') {
    if (!L.supported.checked || t.setup.needsPrepare) return;
    request.supported = true;
    // Laptop-Stop platforms finish Home here and stand through the operator link.
    request.home_only = L.homeModeSelect.value === 'home-only' || (t.fcOperator !== false && t.matchCapable !== false);
  }
  const wantStand = step === 'home' && L.homeModeSelect.value === 'stand';
  const setup = {action: step, homeOnly: request.home_only, wantStand, busy: true, message: 'Connecting to the Pi…',
    discovered: t.setup.discovered, needsPrepare: t.setup.needsPrepare};
  t.setup = setup;
  t.feedback = '';
  let completed = false;
  const fail = detail => {
    if (t.setup !== setup) return;
    Object.assign(setup, {message: detail, failed: true, cue: null, discovered: false});
    render();
  };
  render();
  try {
    if (!setup.discovered) await discoverTeam(t.slot);
    if (t.setup !== setup) return;
    setup.discovered = true;
    const socket = new WebSocket('ws://' + location.host + '/setup/' + t.slot + '?owner=' + owner);
    setup.socket = socket;
    socket.onopen = () => { if (t.setup === setup) socket.send(JSON.stringify(request)); };
    socket.onmessage = ({data}) => {
      if (t.setup !== setup) return;
      const event = JSON.parse(data);
      if (event.event !== 'setup') return;
      setup.message = event.message || '';
      if (event.stage === 'stand_prompt' && step === 'home' && !request.home_only) {
        setup.cue = event.cue;
      } else if (event.stage === 'home_startup_fault') {
        setup.cue = null;
        setup.needsPrepare = true;
      } else if (event.stage === 'complete') {
        completed = event.action === step;
        if (step === 'prepare' && completed) { setup.prepared = true; setup.needsPrepare = false; }
        t.fcOperator = event.fc_operator_control === true;
        t.homed = event.homed === true || (completed && step === 'home');
        if (typeof event.match_capable === 'boolean') t.matchCapable = event.match_capable;
        setup.cue = null;
        setup.connectAfterHome = completed && t.matchCapable !== false && step === 'home' && !request.home_only && event.home_only === false;
      } else if (event.stage === 'error') {
        fail(event.message || 'The Pi reported a setup error.');
      }
      render();
    };
    socket.onerror = () => fail('Could not reach the robot setup session.');
    socket.onclose = () => {
      if (t.setup !== setup) return;
      setup.busy = false;
      setup.socket = null;
      setup.cue = null;
      if (step === 'home') L.supported.checked = false;
      if (!completed && !setup.failed) fail('Setup connection lost. Keep Pupper supported and check the Pi before retrying.');
      if (completed && t.fcOperator && t.homed && (wantStand || (step === 'prepare' && L.homeModeSelect.value === 'stand'))) {
        openOperator(t);
      } else if (completed && wantStand && request.home_only && !t.fcOperator) {
        setup.message = 'Homed on the stand. This Pi uses a physical Stop gamepad. Connect it and select Home again to enable standing.';
      } else if (setup.connectAfterHome) {
        void t.openRunner(true);
      }
      render();
    };
  } catch (e) {
    setup.busy = false;
    if (step === 'home') L.supported.checked = false;
    fail(e.message);
  }
}

/* ---------- laptop Stop link and deliberate Stand ---------- */
function stopOperator(t) {
  const op = t.operator;
  if (!op || op.stopping) return;
  op.stopping = true;
  op.ready = op.standing = false;
  if (op.socket.readyState === WebSocket.OPEN) op.socket.send(JSON.stringify({action: 'stop'}));
  else op.socket.close();
}

function openOperator(t) {
  if (t.operator || !t.fcOperator || !t.homed || document.hidden || t.online || t.setup.busy || replay()) return;
  const socket = new WebSocket('ws://' + location.host + '/operator/' + t.slot + '?owner=' + owner);
  const op = t.operator = {socket, ready: false, standing: false, waiting: false, offer: null, stopping: false};
  function checkConnection() {
    if (op.result || !op.offer || freshOperatorOffer(op.offer)) return;
    op.lost = true;
    op.result = 'Stop link lost. Stop unconfirmed. Check Pupper.';
    stopOperator(t);
    clearKeys(t.slot);
    t.ready = false;
    t.feedback = t.setup.message = op.result;
    if (t.socket?.readyState === WebSocket.OPEN) t.socket.send(JSON.stringify({command: 'stop', robot_id: t.id}));
    render();
    // Let the Pi deliver its Stop result, then release the dead link. Reconnecting is a new action.
    setTimeout(() => { if (t.operator === op) socket.close(); }, 2000);
  }
  // The state poll can stall independently of this Stop link, so it has its own watchdog.
  const watchdog = setInterval(checkConnection, 100);
  t.setup.message = 'Linking laptop Stop to the Pi…';
  socket.onmessage = ({data}) => {
    if (t.operator !== op) return;
    const event = JSON.parse(data);
    if (event.event === 'operator_challenge') {
      checkConnection();
      if (document.hidden) { stopOperator(t); return; }
      if (op.stopping || socket.readyState !== WebSocket.OPEN || socket.bufferedAmount) return;
      const words = event.challenge.split(' ');
      if (words.length !== 3 || !/^\d+$/.test(words[2])) return;
      const identity = words.slice(0, 2).join(' '), sequence = BigInt(words[2]);
      if (op.offer?.identity === identity && sequence <= op.offer.sequence) return;
      if (op.offer && op.offer.identity !== identity) t.ui.life.supported.checked = false;
      op.offer = {challenge: event.challenge, identity, sequence, received: performance.now(), wall: Date.now()};
      socket.send(JSON.stringify({action: 'heartbeat', challenge: event.challenge}));
      return;
    }
    if (event.event === 'operator_status' && !op.stopping) {
      op.ready = event.ready === true;
      if (!op.waiting && !op.standing) t.setup.message = op.ready ?
        'Laptop Stop linked. Confirm Pupper is supported, then enable standing.' : (event.reason || 'Waiting for the Pi Stop gateway…');
    } else if (event.event === 'operator_standing' && !op.stopping) {
      op.ready = op.standing = true;
      op.waiting = false;
      t.setup.message = 'Pi reports the walker standing. Opening match controls…';
      void t.openRunner(true);
    } else if (event.event === 'operator_stop' || event.event === 'error') {
      op.terminal = op.stopping = true;
      op.standing = op.ready = false;
      op.result = event.event === 'error' ? (event.reason || 'Pi operator connection failed.') :
        (event.confirmed ? 'Pi confirmed Stop.' : 'Stop unconfirmed. Check Pupper.') + (event.reason ? ' ' + event.reason : '');
      if (op.runnerError) op.result += ' ' + op.runnerError;
      t.feedback = t.setup.message = op.result;
    }
    render();
  };
  socket.onclose = () => {
    clearInterval(watchdog);
    if (t.operator !== op) return;
    t.operator = null;
    t.homed = false;
    t.setup = {message: op.result || 'Stop link lost. Stop unconfirmed. Check Pupper and reconnect.'};
    t.feedback = t.setup.message;
    clearKeys(t.slot);
    t.socket?.close();
    render();
  };
}

function standClick(t) {
  const setup = t.setup, op = t.operator, L = t.ui.life;
  if (op) {
    if (!op.ready || op.waiting || op.stopping || op.standing || document.hidden || !L.supported.checked) return;
    // The Pi accepts a challenge for 0.30 s. Never send an older one.
    if (!freshOperatorOffer(op.offer, 300) || op.socket.readyState !== WebSocket.OPEN || op.socket.bufferedAmount) {
      setup.message = 'The standing cue expired. Wait for a fresh cue and enable standing again.';
      render();
      return;
    }
    op.waiting = true;
    op.socket.send(JSON.stringify({action: 'stand', challenge: op.offer.challenge, supported: true}));
    setup.message = 'Standing requested. Waiting for the Pi…';
    render();
    return;
  }
  if (!setup?.cue || setup.socket?.readyState !== WebSocket.OPEN) return;
  setup.socket.send(JSON.stringify({action: 'stand', cue: setup.cue}));
  setup.cue = null;
  setup.message = 'Standing cue sent. Waiting for the walker…';
  render();
}

/* ---------- match controls (play socket) ---------- */
async function openRunner(t, alreadyDiscovered = false) {
  if (t.online) { setView('settings'); return; }
  if (t.connecting || t.socket || t.setup.busy || t.savingAddress) return;
  const attempt = Symbol();
  Object.assign(t, {attempt, connecting: true, stopping: false, preparing: false, feedback: ''});
  message(''); render();
  try {
    if (!alreadyDiscovered) await discoverTeam(t.slot);
    if (t.attempt !== attempt) return;
    const lease = t.operator?.standing ? '&lease=1' : '';
    const sock = new WebSocket('ws://' + location.host + '/play/' + t.slot + '?owner=' + owner + lease);
    t.socket = sock; render();
    sock.onmessage = ({data}) => onRunnerEvent(t, sock, JSON.parse(data));
    sock.onerror = () => { t.feedback = 'Could not open the robot session.'; render(); };
    sock.onclose = () => onRunnerClosed(t);
  } catch (e) {
    if (t.attempt !== attempt) return;
    t.feedback = e.message; t.connecting = false; render();
  }
}

function onRunnerEvent(t, sock, event) {
  if (event.event === 'preparing') { t.preparing = true; render(); return; }
  if (event.event === 'profile' || event.event === 'home_required') {
    t.matchCapable = event.match_capable !== false;
    t.imuProfile = (event.imu_recovery || 'strict').replaceAll('_', ' ');
    if (event.event === 'home_required') {
      t.needsHome = true;
      t.fcOperator = event.fc_operator_control === true;
      t.homed = event.homed === true;
      t.previousFault = event.previous_fault || null;
      t.joystickListed = event.joystick_listed;
    }
    return;
  }
  if (event.event === 'onboard_ball' && event.robot_id === t.id && event.onboard_ball) {
    t.ball = event.onboard_ball;
    t.ballReceived = performance.now();
    if (!t.requestedBall && ['pink', 'confirmed', 'off'].includes(t.ball.mode)) t.ui.ts.ballMode.value = t.ball.mode;
    return;
  }
  if (event.event === 'stop_result') {
    t.feedback = event.synthetic ? 'Simulated Stop recorded. No physical Stop occurred.' :
      event.confirmed ? 'Robot acknowledged Stop.' : 'Stop was not confirmed. Check the robot.';
    render();
    return;
  }
  if (event.event !== 'controls') {
    if (t.operator?.terminal) return;
    if (t.operator?.lost) {
      if (event.event === 'stop_result' && event.confirmed) t.feedback = t.setup.message = 'Stop link lost. Pi confirmed Stop through match controls.';
      return;
    }
    if ((event.event === 'error' || event.accepted === false) && event.reason) {
      if (event.event === 'error' && t.operator) {
        t.operator.runnerError = event.reason;
        if (t.operator.result) t.operator.result += ' ' + event.reason;
      }
      t.feedback = t.operator?.result || event.reason;
      render();
    }
    return;
  }
  if (t.stopping || t.operator?.stopping) return;
  t.imu = event.imu;
  if (t.imu?.state !== 'healthy') down.clear();
  if (t.mode !== event.mode || !event.ready) clearKeys(t.slot);
  const first = !t.online;
  Object.assign(t, {online: true, connecting: false, preparing: false, ready: event.ready, mode: event.mode, synthetic: event.synthetic === true});
  t.ui.ts.mode.value = t.mode;
  let command = null;
  if (t.requestedBall) {
    command = {command: 'onboard_ball_mode', mode: t.requestedBall};
    t.requestedBall = null;
  } else if (t.requested) {
    if (t.requested.expires >= performance.now()) command = {command: 'mode', mode: t.requested.mode};
    t.requested = null;
  } else if (t.mode === 'teleop' && state.phase === 'playing' && state.camera?.match?.epoch && !pending) {
    if (!state.camera.live || state.camera.match.phase !== 'playing') clearKeys(t.slot);
    command = {command: 'drive', epoch: state.camera.match.epoch, ...readDrive(down, t.slot)};
  }
  if (command && sock.readyState === WebSocket.OPEN && sock.bufferedAmount === 0) {
    sock.send(JSON.stringify({...command, robot_id: t.id, ticket: event.ticket}));
  }
  if (first) {
    if (t.operator?.standing) t.setup.message = '';
    render();
  }
}

function onRunnerClosed(t) {
  Object.assign(t, {socket: null, online: false, connecting: false, preparing: false, ready: false, requested: null, mode: null, ball: null, requestedBall: null});
  clearKeys(t.slot);
  t.setup = {message: ''};
  if (t.needsHome) {
    t.needsHome = false;
    t.needsHomeShown = true;
    t.setup = {prepared: true, discovered: true, message: t.matchCapable ?
      'Connected. Support Pupper on the stand, then select Home.' : 'This Pi profile is not match capable. Keep Pupper supported.'};
    if (t.previousFault) t.setup.message = 'The previous session reported a board fault: ' + t.previousFault +
      '. Inspect Pupper and support it before requesting a fresh Home.';
    if (t.fcOperator && t.homed) {
      t.setup.message = 'Pi reports Home complete. Confirm Pupper is supported, then enable standing.';
      openOperator(t);
    }
    setView('robot');
  } else if (t.operator) {
    stopOperator(t);
  } else if (!t.feedback) t.feedback = 'Session closed. Check power, Wi-Fi and Home if this was unexpected.';
  render();
}

/* ---------- Stop ---------- */
function stopAll() {
  down.clear();
  for (const t of Object.values(teams)) {
    stopOperator(t);
    cancelSetup(t);
    t.requested = null;
    t.attempt = null;
    if (t.connecting && !t.socket) t.connecting = false;
    if (t.socket?.readyState === WebSocket.OPEN) {
      t.stopping = true; t.ready = false; t.feedback = 'Stop requested…';
      t.socket.send(JSON.stringify({command: 'stop', robot_id: t.id}));
    } else if (t.socket) t.socket.close();
  }
  void action('match/end');
}
$('stop-all').onclick = stopAll;
window.addEventListener('keydown', e => {
  if (e.code === 'Escape') { if (!e.repeat) stopAll(); return; }
  const t = teams[slotForKey(e.code)];
  if (state.phase !== 'playing' || !state.camera?.live || state.camera.match?.phase !== 'playing' || !t?.ready || t.stopping || t.mode !== 'teleop'
      || e.metaKey || e.ctrlKey || e.altKey || e.target?.matches('input,select,textarea,[contenteditable="true"]')) return;
  e.preventDefault(); if (!e.repeat) down.add(e.code);
});
window.addEventListener('keyup', e => down.delete(e.code));
window.addEventListener('blur', () => down.clear());
document.addEventListener('visibilitychange', () => { if (document.hidden) { down.clear(); for (const t of Object.values(teams)) stopOperator(t); } });
window.addEventListener('pagehide', () => { down.clear(); for (const t of Object.values(teams)) { stopOperator(t); cancelSetup(t); t.socket?.close(); } });

/* ---------- match actions ---------- */
$('home-play').onclick = () => setView('play');
$('start-match').onclick = () => action('match/start');
$('end-match').onclick = () => { down.clear(); return action('match/end'); };
$('resume-balance').onclick = () => { down.clear(); return action('match/balance-resume', {id: state.balance?.id}); };
$('resume-match').onclick = () => action('match/resume');
$('place-ball').onclick = () => action('replay/place-ball');
function kickoff() {
  const live = Object.values(teams).filter(t => t.online), cam = state.camera;
  const unverified = live.find(t => t.imu?.state !== 'healthy' || !t.imu?.board_session ||
    !Number.isInteger(t.imu?.generation) || t.imu.generation < 1 || !Number.isInteger(t.imu.incident) || t.imu.incident < 0);
  const visible = cam?.visible || {};
  const missing = [];
  if (!live.length) missing.push('connect a robot');
  if (!cam) missing.push('connect the field camera');
  else if (!cam.live || !cam.frame?.calibration_valid) missing.push('calibrate the field');
  else {
    if (!visible.ball) missing.push('place the ball in view');
    const hidden = live.filter(t => !visible.robots?.[TRACK[t.slot]]);
    if (hidden.length) missing.push('put ' + hidden.map(t => t.name).join(' and ') + ' in view');
  }
  // The server measures what each field play will refuse at kickoff: ball and robots inside the field.
  for (const problem of state.kickoff?.field || [])
    missing.push(replay() && problem.startsWith('the ball') ? problem + ' — use Place ball at centre on the match screen' : problem);
  if (unverified) missing.push(unverified.name + ': ' + (unverified.imu?.reason || 'current board feedback required'));
  if (live.some(t => t.stopping)) missing.push('reconnect stopped robots');
  if (Object.values(teams).some(t => t.setup.busy)) missing.push('finish robot setup');
  return {live, missing, canStart: !missing.length};
}

/* ---------- rendering ---------- */
function render() {
  const cam = state.camera, match = cam?.match;
  if (state.phase !== 'playing' || lastDriveEpoch !== match?.epoch) down.clear();
  lastDriveEpoch = match?.epoch;
  renderApp();
  const k = kickoff();
  renderHeader(match);
  renderHome(k);
  renderLifecycles();
  renderSettings();
  renderPlay(k);
  renderField();
  updateFrames();
}

const PHASE_TEXT = {idle: 'Waiting', starting: 'Kickoff starting', playing: 'Playing', stopping: 'Stopping', balance_hold: 'Paused for balance', balance_resuming: 'Resuming'};
function renderHeader(match) {
  const score = match?.score || state.score || {};
  setText($('score-blue'), String(score['pupper-1'] ?? 0));
  setText($('score-red'), String(score['pupper-2'] ?? 0));
  const phase = state.phase === 'playing' && match?.phase === 'goal' ? 'Goal' : PHASE_TEXT[state.phase] || state.phase;
  setText($('match-chip'), state.phase === 'idle' ? 'No match' : 'Match · ' + phase + ' · ' + (score['pupper-1'] ?? 0) + '–' + (score['pupper-2'] ?? 0));
  setText($('match-phase'), phase);
}

function renderHome(k) {
  for (const t of Object.values(teams)) {
    const K = t.ui.kit, [text, tone] = teamStatus(t);
    K.root.dataset.online = String(t.online);
    setText(K.kitStatus, text); K.kitStatus.dataset.tone = tone;
    setText(K.kitNote, t.feedback || (replay() ? 'Simulated robot · ' + t.id : t.id));
    const label = t.online ? 'Robot setup' : (t.setup.prepared || t.homed || t.operator || t.needsHomeShown || t.setup.busy) ? 'Continue setup' : 'Connect ' + t.name;
    setText(K.kitConnect, label);
    K.kitConnect.disabled = !t.online && (t.connecting || t.savingAddress || (state.phase !== 'idle' && !t.setup.busy));
  }
  const cam = state.camera;
  setText($('camera-status'), cam ? (cam.synthetic ? 'Simulated · ' : '') + cam.status.replace(/^Synthetic camera · /, '') : 'Disconnected');
  $('camera-status').dataset.tone = cam?.live && cam.frame?.calibration_valid ? 'good' : cam ? 'warn' : '';
  setText($('camera-connect'), cam?.frame ? 'Field setup' : replay() ? 'Open simulated field' : 'Connect camera');
  setText($('home-ready'), state.phase !== 'idle' ? 'A match is open. Go to the match screen to follow it.' :
    k.canStart ? 'Ready for kickoff.' : 'Before kickoff: ' + k.missing.join('; ') + '.');
}

function setStep(el, value) { el.dataset.state = value; }
function renderLifecycles() {
  for (const t of Object.values(teams)) {
    const L = t.ui.life, setup = t.setup, op = t.operator, idle = state.phase === 'idle';
    const [text, tone] = teamStatus(t);
    setText(L.lifeStatus, text); L.lifeStatus.dataset.tone = tone;
    const locked = setup.busy || t.savingAddress || !idle;
    L.addressInput.disabled = L.saveAddress.disabled = locked || t.connecting || t.online || Boolean(op) || replay();
    L.connect.disabled = t.online || t.connecting || setup.busy || t.savingAddress || Boolean(op) || !idle;
    L.prepare.disabled = locked || setup.prepared || t.online || t.connecting || Boolean(op) || replay();
    L.disconnect.hidden = !(t.online || op);
    if (setup.busy && setup.action === 'home') L.supported.checked = true;
    L.supported.disabled = locked && !op;
    L.homeModeSelect.disabled = locked || Boolean(op);
    const laptopStand = t.fcOperator && t.homed && L.homeModeSelect.value === 'stand';
    L.home.hidden = Boolean(setup.cue || laptopStand);
    L.home.disabled = locked || setup.needsPrepare || !L.supported.checked || t.online || t.connecting || replay();
    L.stand.hidden = !setup.cue && !laptopStand;
    L.stand.disabled = laptopStand ? (!op?.ready || op.standing || op.waiting || op.stopping || !freshOperatorOffer(op.offer, 300) || !L.supported.checked) : !setup.cue;
    setText(L.stand, t.matchCapable === false ? 'Enable bench test' : 'Enable standing');
    let link = '';
    if (replay()) link = 'Replay skips Home and standing. Connect opens simulated match controls directly.';
    else if (op) link = op.standing ? 'Pi reports the walker active and owned by this window.' :
      op.ready ? 'Laptop Stop linked and current.' : op.stopping ? (op.result || 'Stopping…') : 'Linking laptop Stop…';
    else if (t.fcOperator === false) link = 'This Pi uses a physical Stop gamepad. Standing follows Home in the same step.';
    else if (t.homed) link = 'Choose "Continue to standing" to link laptop Stop.';
    setText(L.linkState, link);
    setText(L.playState, t.online ? (t.ready ? 'Match controls open' : 'Match controls connected, not ready') + (t.operator?.standing ? ' through this window\'s Stop link.' : '.') :
      'Opens automatically after standing.');
    setStep(L.sConnect, t.online || t.setup.prepared || t.homed || op || t.needsHomeShown ? 'done' : 'active');
    setStep(L.sHome, t.homed || t.online ? 'done' : (t.setup.prepared || t.needsHomeShown) ? 'active' : '');
    setStep(L.sStand, op?.standing || t.online ? 'done' : t.homed || op ? 'active' : '');
    setStep(L.sPlay, t.online ? 'done' : op?.standing ? 'active' : '');
    const detail = setup.message || t.feedback || '';
    setText(L.lifeMessage, detail);
  }
}

function renderSettings() {
  for (const t of Object.values(teams)) {
    const S = t.ui.ts;
    S.mode.disabled = !t.online || state.phase !== 'idle';
    S.ballMode.disabled = !t.online;
    S.guide.hidden = t.mode !== 'teleop';
    setText(S.imuProfile, t.imuProfile ? 'IMU fault handling: ' + t.imuProfile : '');
    setText(S.feedback, t.feedback || '');
    let reading = t.online ? 'Waiting for onboard camera telemetry.' : 'Connect to read the onboard camera.';
    if (t.ball) {
      const elapsed = performance.now() - t.ballReceived;
      const stale = elapsed > 1000 || (t.ball.state === 'tracked' && (!Number.isFinite(t.ball.age_ms) || t.ball.age_ms + elapsed >= 350));
      const bearing = Number.isFinite(t.ball.bearing_deg) ? ', about ' + Math.abs(t.ball.bearing_deg).toFixed(0) + '° ' + (t.ball.bearing_deg < 0 ? 'left' : 'right') : '';
      reading = stale ? 'Onboard observation expired.' : t.ball.state === 'tracked' ? 'Ball tracked by ' + t.ball.source + bearing : 'Onboard camera: ' + t.ball.state;
    }
    setText(S.ballReading, reading);
  }
  const cam = state.camera;
  setText($('settings-camera'), cam ? (cam.synthetic ? 'Rendered simulation. ' : 'Source ' + state.source + '. ') + cam.status +
    (replay() && state.replay?.timing ? ' ' + state.replay.timing : '') : 'No camera open.');
}

const PLANNER_TEXT = {
  idle: ['Not armed', 'hold'], approach: ['Approaching the ball', ''], align: ['Lining up the paw', ''], kick_ready: ['Ready to kick', ''],
  kick_in_flight: ['Kick in progress', ''], kick_cancelling: ['Cancelling kick', 'hold'], kick_complete: ['Kick finished', ''],
  kick_failed: ['Kick failed', 'bad'], kick_cancelled: ['Kick cancelled', 'hold'], kick_hold: ['Kick held', 'hold'], kick_owned: ['Kick owns the legs', ''],
  return_home: ['Returning home after the goal', ''], await_reset: ['Home, waiting for the ball reset', 'hold'], target_reached: ['At target', ''],
  move: ['Moving', ''], defend: ['Defending', ''], blocked: ['Blocked', 'hold'], reassess: ['Reassessing', 'hold'],
  tracking_lost: ['Holding: no current field frame', 'hold'], intent_lost: ['Holding: no current lease', 'hold'],
  participant_hold: ['Holding: a participant is missing', 'hold'], tactical_hold: ['Holding', 'hold'], external_hold: ['Holding: robot guard', 'hold'],
  obstacle_hold: ['Holding: front camera', 'hold'], recovery_required: ['Recovery required', 'bad'], stopped: ['Stopped', 'bad'],
};
const LEASE_TEXT = {attack: 'Attack', defend: 'Defend', move: 'Move'};

function participants() {
  if (state.players?.length) return state.players.map(r => SLOT_OF[r]).filter(Boolean);
  return Object.values(teams).filter(t => t.online).map(t => t.slot);
}

function renderPlay(k) {
  const cam = state.camera, match = cam?.match, playing = state.phase === 'playing';
  const shown = new Set(participants());
  for (const t of Object.values(teams)) {
    const on = shown.has(t.slot);
    t.ui.onboard.root.hidden = t.ui.tele.root.hidden = !on;
    if (!on) continue;
    setText(t.ui.onboard.frame, t.online || state.teams?.[t.id] ? 'No onboard feed' : 'Not connected');
    setText(t.ui.onboard.caption, t.name + ' onboard camera. No onboard stream is available from this robot software yet.');
    renderTelemetry(t, cam);
  }
  $('start-match').hidden = ['playing', 'balance_hold', 'balance_resuming', 'stopping'].includes(state.phase);
  $('start-match').disabled = pending || state.phase !== 'idle' || !k.canStart;
  setText($('start-match'), replay() ? 'Start simulated match' : 'Start match');
  $('end-match').hidden = !['starting', 'playing', 'balance_hold', 'balance_resuming'].includes(state.phase);
  const balance = state.balance;
  $('resume-balance').hidden = !balance || balance.blocked || !['balance_hold', 'balance_resuming'].includes(state.phase);
  $('resume-balance').disabled = pending || !balance?.ready || state.phase !== 'balance_hold';
  $('resume-match').hidden = !playing || !match?.reset_ready;
  $('resume-match').disabled = pending || !state.resume_ready;
  // Replay's stand-in for an operator replacing the ball: between matches, at a goal, or after it leaves the field.
  $('place-ball').hidden = !(replay() && state.replay?.place_ball && !(playing && match?.phase === 'goal' && match.reset_ready));
  $('place-ball').disabled = pending;
  let text = lastError, tone = '';
  if (!text) {
    if (balance) {
      text = [balance.ready ? 'Match paused. Balance feedback restored.' : balance.reason,
        ...Object.entries(balance.teams).map(([id, detail]) => NAMES[SLOT_OF[id]] + ': ' + detail)].join(' ');
      if (!balance.blocked && !balance.camera_ready) text += ' Place the ball and all robots inside the calibrated field in view.';
      tone = 'hold';
    } else if (Object.values(teams).some(t => t.stopping)) {
      text = 'Stopped. Reconnect each stopped robot before a new match.'; tone = 'hold';
    } else if (playing && match?.phase === 'goal') {
      text = match.reset_ready ? (state.resume_ready ? 'Ball in place. Ready to resume.' : 'Ball in place. Waiting for robots to reach home.') :
        replay() ? 'Goal. Place the ball at centre to reset.' : 'Goal. Return the ball to the field.';
      tone = 'hold';
    } else if (state.message) {
      text = state.message; tone = /held|hold|Stop/i.test(state.message) ? 'hold' : '';
    } else {
      text = k.canStart ? 'Field ready for kickoff.' : 'Before kickoff: ' + k.missing.join('; ') + '.';
    }
  }
  setText($('error'), text);
  $('error').dataset.tone = tone;
  setText($('play-wait'), cam ? (replay() ? 'Waiting for the rendered field' : 'Waiting for a camera frame') : 'Connect the field camera to see the pitch.');
}

function renderTelemetry(t, cam) {
  // Server state covers teams owned by another window; this window adds its own socket view.
  const T = t.ui.tele, team = state.teams?.[t.id], reported = team?.telemetry;
  const awaitingCurrent = state.phase === 'starting' || Boolean(reported?.epoch && state.epoch && reported.epoch !== state.epoch);
  const tel = awaitingCurrent ? null : reported, planner = tel?.planner || {};
  const connected = Boolean(team) || t.online, mode = team?.mode || t.mode, synthetic = team?.synthetic ?? t.synthetic;
  setText(T.teleMode, !connected ? 'Disconnected' : (mode === 'teleop' ? 'Keyboard teleop' : 'Autonomous') +
    (synthetic ? ' · simulated robot' : '') + (team && !t.online ? ' · another window' : ''));
  const stale = team?.telemetry_age_s == null || team.telemetry_age_s > 1.5;
  let phase, reason, ptone = '';
  if (!connected) {
    [phase, reason, ptone] = ['Not connected', '', 'hold'];
  } else if (awaitingCurrent) {
    [phase, reason, ptone] = ['Waiting for this match', 'Waiting for current robot telemetry.', 'hold'];
  } else if (state.phase === 'idle' && tel?.stopped === false && planner.phase === 'stopped') {
    [phase, reason] = ['Standby', 'Match ended. Start a new match to arm this runner.'];
  } else if (mode === 'teleop') {
    [phase, reason] = ['Driven from this keyboard', KEY_GUIDE[t.slot]];
  } else if (!tel) {
    [phase, reason, ptone] = ['No planner telemetry', 'This robot\'s runner does not report executor decisions to the laptop.', 'hold'];
  } else {
    const key = planner.phase || tel.phase;
    [phase, ptone] = PLANNER_TEXT[key] || [String(key || 'Unknown').replaceAll('_', ' '), ''];
    reason = planner.reason || tel.reason || '';
    // The executor's own read-only reset check, reported by the replay runner while waiting.
    if (planner.resume) reason += ' · Reset check: ' + (planner.resume === 'ready' ? 'robot ready to resume' : planner.resume);
    if (planner.role) phase += planner.role === 'defense' ? ' (defense)' : '';
    if (stale) { phase = 'Telemetry stale: ' + phase; ptone = 'hold'; }
  }
  setText(T.telePhase, phase); T.telePhase.dataset.tone = ptone;
  setText(T.teleReason, reason || '');
  const row = cam?.frame?.robots?.[TRACK[t.slot]];
  const measured = row && row.measured_this_frame;
  setText(T.tPose, measured ? `${fmt(row.x)}, ${fmt(row.y)} m · ${deg(row.yaw_at_capture ?? row.yaw)}` : row?.visible ? 'Predicted only' : 'Not measured');
  setText(T.tTarget, planner.target ? `${fmt(planner.target[0])}, ${fmt(planner.target[1])} m` + (Number.isFinite(planner.target_heading) ? ' · ' + deg(planner.target_heading) : '') : '–');
  setText(T.tError, Number.isFinite(planner.position_error_m) || Number.isFinite(planner.heading_error_rad) ?
    `${Number.isFinite(planner.position_error_m) ? Math.round(planner.position_error_m * 100) + ' cm' : '–'} · ${deg(planner.heading_error_rad)}` : '–');
  const c = planner.command;
  setText(T.tCommand, c ? `${fmt(c.vx)} / ${fmt(c.vy)} m/s · ${fmt(c.wz)} rad/s` : '–');
  const age = tel?.observed?.capture_age_ms;
  setText(T.tAge, Number.isFinite(age) ? Math.round(age) + ' ms' + (stale ? ' (stale)' : '') : '–');
  const ing = tel?.ingress;
  setText(T.tFrames, ing ? `${ing.frames} admitted` + (ing.rejected ? ` · ${ing.rejected} refused` : '') : '–');
  const lease = team?.field;
  setText(T.tLease, !lease ? (mode === 'teleop' ? 'Keyboard; no field lease' : '–') :
    (lease.paired ? (LEASE_TEXT[lease.kind] || 'Leased') : lease.ready_to_arm ? 'Kickoff ready' : 'Held') + (lease.reason ? ' · ' + lease.reason : ''));
  renderKick(T.tKick, reported?.kick, team?.kick, state.epoch, state.phase === 'starting');
  setText(T.tSource, tel?.labels ? tel.labels.planner + '. ' + tel.labels.native + '.' : tel ? 'Reported by the robot runner.' : '');
}

function renderKick(el, kick, edge, epoch, starting) {
  const view = kickPresentation(kick, edge, epoch, starting);
  const key = JSON.stringify(view);
  if (el.dataset.key === key) return;
  el.dataset.key = key;
  if (!view) { el.replaceChildren(); return; }
  const head = document.createElement('div');
  head.textContent = view.heading;
  if (view.epoch) head.title = 'Match ' + view.epoch;
  const list = document.createElement('ol');
  for (const r of view.receipts) {
    const item = document.createElement('li');
    item.textContent = r.text;
    if (r.synthetic) item.className = 'synthetic';
    list.append(item);
  }
  el.replaceChildren(head, list);
}

function renderField() {
  const cam = state.camera, visible = cam?.visible || {};
  loadCameraForm();
  if (frozen && (frozen.source_session !== cam?.source_session || frozen.revision !== cam?.frame?.calibration_revision)) clearSnapshot();
  $('reset-goals').hidden = !(state.goal_lines && cam?.live && state.phase === 'idle');
  $('survey').disabled = !cam?.live || !cam?.frame?.calibration_valid || state.phase !== 'idle' || Boolean(cam?.synthetic);
  $('apply-camera').disabled = pending || state.phase !== 'idle';
  $('field-snapshot').disabled = !cam?.frame || state.phase !== 'idle';
  setText($('field-snapshot'), frozen ? (replay() ? 'Return to simulated view' : 'Return to live view') : 'Take snapshot');
  $('far-corner').hidden = !(frozen?.mode === 'corner' && frozen.ids.includes(state.camera_settings?.anchor?.tag_id) && state.phase === 'idle');
  $('field-confirm').hidden = !(frozen?.fit_valid && cam?.needs_confirm && state.phase === 'idle');
  setText($('preview-wait'), replay() ? 'Waiting for the rendered field' : 'Waiting for a camera frame');
  const geometry = frozen || cam?.geometry;
  const ids = frozen?.ids || visible.tags || [];
  const fieldIds = ids.filter(i => geometry?.mode === 'corner' ? i === state.camera_settings?.anchor?.tag_id : i >= 0 && i <= 5);
  const robotStatus = Object.entries(cam?.frame?.robots || {}).map(([name, rb]) => {
    const team = name === 'striker' ? 'Blue' : 'Red', detail = geometry?.robot_status?.[name];
    return detail ? team + ': ' + detail : geometry?.robots_seen?.[name] ? team + ' visible' : '';
  }).filter(Boolean);
  setText($('seen'), cam?.frame ? [(geometry?.mode === 'corner' ? 'Corner tag ' : 'Floor tags ') + (fieldIds.join(', ') || 'none'),
    geometry?.ball_seen ? 'ball visible' : 'searching for the ball', ...robotStatus].join('. ') + '.' : '');
  setText($('layout-help'), geometry?.mode === 'layout' ? geometry.layout_guide : '');
  setText($('goal-source'), geometry ? 'Goal lines and mouth: ' + geometry.goal_source + '.' : '');
}

function updateFrames() {
  const cam = state.camera, view = currentView();
  const id = view === 'play' ? 'play-view' : view === 'field' ? 'view' : null;
  for (const [img, wait] of [['view', 'preview-wait'], ['play-view', 'play-wait']]) {
    if (!cam?.frame) { $(img).hidden = true; $(wait).hidden = false; frameKeys[img] = ''; }
  }
  if (!id || !cam?.frame || (id === 'view' && frozen)) return;
  const key = cam.source_session + '/' + cam.frame.seq;
  if (key !== frameKeys[id]) { frameKeys[id] = key; $(id).src = '/api/frame.jpg?t=' + encodeURIComponent(key); }
}
for (const [img, wait] of [['view', 'preview-wait'], ['play-view', 'play-wait']]) {
  $(img).onload = () => { $(img).hidden = false; $(wait).hidden = true; };
  $(img).onerror = () => { $(img).hidden = true; $(wait).hidden = false; };
}

/* ---------- field snapshot, corner and goal survey ---------- */
function clearSnapshot() { frozen = null; survey = null; pickingFar = false; frameKeys.view = ''; $('survey-help').textContent = ''; }
async function takeSnapshot() {
  const shot = await post('camera/snapshot');
  frozen = shot; survey = null; pickingFar = false; $('view').src = shot.image;
  const fit = shot.metrics?.reference_error_px, sensitivity = shot.metrics?.far_sensitivity_m;
  const info = ['Snapshot ' + shot.seq + '.', shot.status + '.', 'Lens: ' + shot.lens + '.'];
  if (Number.isFinite(fit)) info.push('Reference error ' + fit.toFixed(1) + ' px.');
  if (Number.isFinite(sensitivity)) info.push('Far-corner sensitivity ' + (sensitivity * 100).toFixed(1) + ' cm.');
  if (shot.mode === 'corner' && shot.fit_valid) info.push('Confirm only if each physical line intersection lies inside its labelled 7.5 cm ring and both end lines align.');
  if (shot.mode === 'corner' && !shot.fit_valid && shot.ids.includes(state.camera_settings?.anchor?.tag_id)) info.push('Mark the diagonally opposite floor-line intersection to add a distant reference.');
  $('survey-help').textContent = info.join(' ');
  render();
  return shot;
}
$('field-snapshot').onclick = async () => {
  try { if (frozen) { clearSnapshot(); render(); } else await takeSnapshot(); }
  catch (e) { message(e.message); }
};
$('far-corner').onclick = () => {
  if (!frozen) return;
  const opposite = {west_north: 'southeast', east_north: 'southwest', west_south: 'northeast', east_south: 'northwest'};
  pickingFar = true; survey = null;
  $('survey-help').textContent = 'Click the ' + opposite[state.camera_settings.anchor.corner] + ' line intersection on the floor. Use a distinct tape cross if the lines are unclear.';
};
$('field-confirm').onclick = async () => {
  if (!frozen) return;
  await action('camera/confirm', {token: frozen.token});
  if (!lastError) { clearSnapshot(); render(); }
};
function openCamera() {
  clearSnapshot();
  const field_mode = $('field-mode').value, anchor = {corner: $('anchor-corner').value};
  if (field_mode === 'corner') for (const [key, id] of Object.entries(anchorIds)) {
    if ($(id).value.trim() === '' || !Number.isFinite($(id).valueAsNumber)) { message('Enter ' + $(id).parentElement.textContent.trim() + '.'); return; }
    anchor[key] = $(id).valueAsNumber;
  }
  const patch_height = $('patch-height').valueAsNumber;
  if ($('robot-tracking').value === 'colors' && !Number.isFinite(patch_height)) { message('Enter the height of the patch surfaces above the floor.'); return; }
  return action('camera', {source: $('camera-source').value, appearance: $('appearance').checked, patch_height,
    field_mode, robot_tracking: $('robot-tracking').value, anchor});
}
$('apply-camera').onclick = openCamera;
$('self-test').onclick = async () => {
  const button = $('self-test'), result = $('self-test-result');
  button.disabled = true;
  result.textContent = 'Checking the lobby with a simulated field…';
  try {
    const report = await post('self-test');
    result.textContent = 'Software only. ' + (report.ok ? report.summary : (report.summary || 'Offline check failed.')) + ' Hardware not qualified.';
  } catch (e) { result.textContent = e.message; }
  finally { button.disabled = false; }
};
const postNames = ['first west post', 'second west post', 'first east post', 'second east post'];
$('reset-goals').onclick = () => action('goals/configured');
$('survey').onclick = async () => {
  if (survey) { clearSnapshot(); render(); return; }
  try {
    const shot = frozen || await takeSnapshot();
    survey = {points: [], session: shot.source_session, revision: shot.revision, token: shot.token};
    $('survey-help').textContent = 'Click the base of the ' + postNames[0] + ' in the snapshot.';
  } catch (e) { message(e.message); }
};
$('view').onclick = async e => {
  if (!survey && !pickingFar) return;
  const r = e.currentTarget.getBoundingClientRect();
  if (pickingFar) {
    pickingFar = false;
    await action('camera/far-corner', {token: frozen.token, point: [(e.clientX - r.left) / r.width, (e.clientY - r.top) / r.height]});
    if (!lastError) { clearSnapshot(); render(); }
    return;
  }
  survey.points.push([(e.clientX - r.left) / r.width, (e.clientY - r.top) / r.height]);
  if (survey.points.length < 4) { $('survey-help').textContent = 'Click the base of the ' + postNames[survey.points.length] + '.'; return; }
  const data = survey; survey = null;
  await action('goals', data);
  if (!lastError) clearSnapshot();
  $('survey-help').textContent = lastError || 'Goalposts saved for this field.';
};

/* ---------- boot ---------- */
async function refresh() {
  try {
    const response = await fetch('/api/state', {cache: 'no-store'});
    if (!response.ok) throw new Error('The match server is unavailable');
    state = await response.json();
    if (lastError === 'The match server is unavailable') message('');
    render();
  } catch (e) { message(e.message); }
  setTimeout(refresh, 280);
}
try {
  const info = await fetch('/api/app', {cache: 'no-store'});
  if (info.ok) appInfo = await info.json();
} catch {}
try {
  const resp = await fetch('/slots');
  if (!resp.ok) throw new Error('The local match server is unavailable');
  for (const [slot, config] of Object.entries(await resp.json())) addTeam(slot, config);
  $('teams').append($('camera-kit').content.firstElementChild.cloneNode(true));
  wireCameraKit();
  setView(location.hash.slice(1));
  await refresh();
} catch (e) { message(e.message); }

function wireCameraKit() {
  $('camera-connect').onclick = () => { setView('field'); if (!state.camera?.frame && state.phase === 'idle') openCamera(); };
}
