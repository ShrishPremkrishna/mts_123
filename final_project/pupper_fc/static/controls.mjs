import {KEY_GUIDE, readDrive, slotForKey} from './drive.mjs';

const $ = id => document.getElementById(id);
const owner = crypto.randomUUID();
const teams = {}, down = new Set();
let state = {phase:'idle'}, pending = false, survey = null, frameKey = '', lastError = '';
let lastDriveEpoch = null;
let setupTeam = null, frozen = null, pickingFar = false, cameraFormLoaded = false;
const anchorIds = {length:'anchor-length', width:'anchor-width', goal_width:'anchor-mouth',
  tag_size:'anchor-size', rotation:'anchor-rotation', inset_x:'anchor-x', inset_y:'anchor-y', tag_id:'anchor-id'};
function cameraForm() {
  const corner = $('field-mode').value === 'corner', colors = $('robot-tracking').value === 'colors';
  $('anchor-fields').hidden = !corner;
  $('patch-height-field').hidden = !colors;
  $('tracking-help').textContent = colors ? 'Centre a matte 9 cm blue or red patch on the back. Place a 3.6 cm bright lime patch 9 cm forward on the same plane. A missing or ambiguous nose holds play.' :
    'Blue wears tag 10 and Red wears tag 11. Point the printed top toward the nose.';
}
function loadCameraForm(){
  if (cameraFormLoaded || !state.field_defaults) return;
  cameraFormLoaded = true;
  const settings = state.camera_settings || {}, f = state.field_defaults;
  $('field-mode').value = settings.field_mode || 'layout';
  $('robot-tracking').value = settings.robot_tracking || 'tags';
  $('patch-height').value = settings.patch_height ?? .18;
  const values = settings.anchor || {length:f.length, width:f.width, goal_width:f.goal_width, tag_size:state.floor_size};
  for(const [key,id] of Object.entries(anchorIds)) if(values[key] != null) $(id).value = values[key];
  $('anchor-corner').value = values.corner || 'west_north';
  cameraForm();
}
$('field-mode').onchange = $('robot-tracking').onchange = cameraForm;

const message = text => { lastError = text; $('error').textContent = text; };

async function post(path, data = {}) {
  const response = await fetch('/api/' + path, {method:'POST', headers:{'content-type':'application/json','X-Pupper-FC':owner}, body:JSON.stringify(data)});
  const result = await response.json();
  if (!response.ok) throw new Error(result.detail || 'Request failed');
  return result;
}

function clearKeys(slot) {
  for (const key of down) if (slotForKey(key) === slot) down.delete(key);
}
function render() {
  const live = Object.values(teams).filter(t => t.online), cam = state.camera, match = cam?.match;
  if(state.phase !== 'playing' || lastDriveEpoch !== match?.epoch) down.clear();
  lastDriveEpoch=match?.epoch;
  $('score-blue').textContent = (match?.score || state.score)?.['pupper-1'] ?? 0;
  $('score-red').textContent = (match?.score || state.score)?.['pupper-2'] ?? 0;
  $('camera-status').textContent = cam ? ((state.source === 'sim' ? 'Simulated · ' : '') + cam.status) : 'Disconnected';
  $('camera-connect').textContent = cam?.frame ? 'View field' : 'Connect camera';
  $('start-match').hidden = ['playing','balance_hold','balance_resuming','stopping'].includes(state.phase);
  const unverified = live.find(t => t.imu?.state !== 'healthy' || !t.imu?.board_session ||
    !Number.isInteger(t.imu?.generation) || t.imu.generation < 1 || !Number.isInteger(t.imu.incident) || t.imu.incident < 0);
  const visible = cam?.visible || {}, canStart = !unverified && cam?.live && cam?.frame?.calibration_valid && visible.ball &&
        live.length && live.every(t => visible.robots?.[t.slot === 'blue' ? 'striker' : 'keeper']);
  $('start-match').disabled = pending || state.phase !== 'idle' || !canStart ||
    Object.values(teams).some(t => t.setup.busy);
  $('end-match').hidden = !['starting','playing','balance_hold','balance_resuming'].includes(state.phase);
  const balance=state.balance;
  $('resume-balance').hidden = !balance || balance.blocked || !['balance_hold','balance_resuming'].includes(state.phase);
  $('resume-balance').disabled = pending || !balance?.ready || state.phase !== 'balance_hold';
  $('resume-match').hidden = state.phase !== 'playing' || !match?.reset_ready;
  $('resume-match').disabled = pending || !state.resume_ready;
  loadCameraForm();
  if (frozen && (frozen.source_session !== cam?.source_session || frozen.revision !== cam?.frame?.calibration_revision)) clearSnapshot();
  $('reset-goals').hidden = !(state.goal_lines && cam?.live && state.phase === 'idle');
  $('survey').disabled = !cam?.live || !cam?.frame?.calibration_valid || state.phase !== 'idle';
  $('apply-camera').disabled = pending || state.phase !== 'idle';
  $('field-snapshot').disabled = !cam?.frame || state.phase !== 'idle';
  $('field-snapshot').textContent = frozen ? 'Return to live view' : 'Take snapshot';
  $('far-corner').hidden = !(frozen?.mode === 'corner' && frozen.ids.includes(state.camera_settings?.anchor?.tag_id) && state.phase === 'idle');
  $('field-confirm').hidden = !(frozen?.fit_valid && cam?.needs_confirm && state.phase === 'idle');
  const geometry = frozen || cam?.geometry;
  const ids = frozen?.ids || visible.tags || [];
  const fieldIds = ids.filter(i => geometry?.mode === 'corner' ? i === state.camera_settings?.anchor?.tag_id : i>=0 && i<=5);
  const idsText = geometry?.mode === 'corner' ? 'Corner tag' : 'Mapped floor tags';
  const robotStatus = Object.entries(cam?.frame?.robots || {}).map(([name, rb]) => {
    const team = name === 'striker' ? 'Blue' : 'Red';
    const detail=geometry?.robot_status?.[name];
    return detail ? team + ' · ' + detail : geometry?.robots_seen?.[name] ? team + ' visible' : '';
  }).filter(Boolean);
  $('seen').textContent = cam?.frame ? [idsText + ' ' + (fieldIds.join(', ') || 'none'), geometry?.ball_seen ? 'Ball visible' : 'Searching for ball', ...robotStatus].join(' · ') : '';
  $('layout-help').textContent = geometry?.mode === 'layout' ? geometry.layout_guide : '';
  $('goal-source').textContent = geometry ? 'Goal lines and mouth: ' + geometry.goal_source + '.' : '';
  for (const t of Object.values(teams)) {
    t.modeEl.disabled = !t.online || state.phase !== 'idle';
    t.ballMode.disabled = !t.online;
    if (t.ball) {
      const elapsed = performance.now() - t.ballReceived;
      const stale = elapsed > 1000 || (t.ball.state === 'tracked' &&
        (!Number.isFinite(t.ball.age_ms) || t.ball.age_ms + elapsed >= 350));
      const bearing = Number.isFinite(t.ball.bearing_deg) ? ' · ~' + Math.abs(t.ball.bearing_deg).toFixed(0) +
        '° camera ' + (t.ball.bearing_deg < 0 ? 'left' : 'right') : '';
      t.ballReading.textContent = stale ? 'Camera observation expired.' :
        t.ball.state === 'tracked' ? 'Ball tracked · ' + t.ball.source + bearing : 'Camera · ' + t.ball.state;
    }
    if (!t.online) {
      t.connect.disabled = t.connecting || t.setup.busy || t.savingAddress || state.phase !== 'idle';
      t.options.disabled = t.connecting || (state.phase !== 'idle' && !t.setup.busy);
      t.options.setAttribute('aria-expanded', String(setupTeam === t));
    }
  }
  renderSetup();
  if (balance) {
    const lines=[balance.ready ? 'Match paused. Balance feedback restored.' : balance.reason, ...Object.entries(balance.teams).map(([id, detail])=>(id==='pupper-1'?'Blue':'Red')+' · '+detail)];
    if(!balance.blocked && !balance.camera_ready)lines.push('Place the ball and all teams inside the original calibrated field in view of the camera.');
    $('error').textContent=lines.join(' · ');
  } else if (state.phase === 'idle' && !lastError && unverified) {
    const name = unverified.slot === 'blue' ? 'Blue' : 'Red';
    $('error').textContent = name + ' · ' + (unverified.imu?.reason || 'Current board IMU feedback required. Check the Pi release and team setup.');
  } else if (!lastError) $('error').textContent = state.phase === 'playing' && match?.phase === 'goal' ?
    (match.reset_ready ? (state.resume_ready ? 'Ball in place. Ready to resume.' : 'Ball in place. Waiting for robots to reach home.') : 'Goal! Return the ball to the field.') : (state.message ||
    (state.source === 'sim' && cam ? 'Field preview is running. Select a live camera to start a match.' :
      canStart ? 'Field ready for kickoff.' : live.length && cam?.live ? 'Place the ball and connected teams in camera view.' : 'Connect a team and the field camera to play.'));
  if (!$('fieldpanel').hidden && cam?.frame && !frozen) {
    const key = cam.source_session + '/' + cam.frame.seq;
    if (key !== frameKey) {
      frameKey = key;
      $('view').src = '/api/frame.jpg?t=' + encodeURIComponent(key);
    }
  } else if (!cam?.frame) {
    $('view').hidden = true;
    $('preview-wait').hidden = false;
  }
}

async function action(path, data) {
  pending = true; message(''); render();
  try { state = await post(path,data); }
  catch(e) { message(e.message); }
  finally { pending = false; render(); }
}

function addressKey(slot) { return 'pupper-fc-address-' + slot; }
function savedAddress(slot) {
  try { return localStorage.getItem(addressKey(slot)) || ''; } catch { return ''; }
}
const discoverTeam = slot => post('discover', {slot, host:savedAddress(slot)});

function showSetup(t) {
  if (setupTeam !== t) {
    $('setup-supported').checked = false;
    $('setup-home-mode').value = 'stand';
    $('setup-address').value = t ? savedAddress(t.slot) : '';
  }
  setupTeam = t;
  $('robot-setup').hidden = !t;
  render();
}

function renderSetup() {
  const t = setupTeam;
  if (!t) return;
  const setup = t.setup;
  const locked = setup.busy || t.savingAddress || state.phase !== 'idle';
  $('setup-address').disabled = $('save-address').disabled = locked || t.connecting || t.online;
  if (setup.busy && setup.action === 'home') {
    $('setup-supported').checked = true;
    $('setup-home-mode').value = setup.homeOnly ? 'home-only' : 'stand';
  }
  $('robot-setup').dataset.team = t.slot;
  $('setup-title').textContent = (t.slot === 'blue' ? 'Blue' : 'Red') + ' Pupper setup';
  $('setup-prepare').disabled = locked || setup.prepared;
  $('setup-supported').disabled = locked;
  $('setup-home-mode').disabled = locked;
  $('setup-home').hidden = Boolean(setup.cue);
  $('setup-home').disabled = locked || !$('setup-supported').checked;
  $('setup-stand').hidden = !setup.cue;
  $('setup-stand').disabled = !setup.cue;
  $('setup-stand').textContent = t.matchCapable === false ? 'Enable bench test' : 'Enable standing';
  if (setup.busy && setup.action === 'home') {
    $('setup-hint').textContent = 'Keep Pupper supported. Use the gamepad for physical Stop. Esc cancels a pending standing cue.';
  } else {
    $('setup-hint').textContent = $('setup-home-mode').value === 'home-only'
      ? 'Home the legs and leave the walker inactive.'
      : t.joystickListed === false ? 'At Connect, the Pi listed no joystick. Home only is available now. Connect the Stop gamepad to enable standing from here.'
      : 'Click Enable standing after Home. Keep the Stop gamepad connected.';
  }
  if (t.matchCapable === false) $('setup-hint').textContent = 'Bench damping profile · keep Pupper supported throughout. Match play is disabled.';
  if ($('setup-status').textContent !== setup.message) $('setup-status').textContent = setup.message;
}

function cancelSetup(t) {
  if (!t.setup.busy) return;
  const socket = t.setup.socket;
  t.setup = {message:'Setup connection closed. If homing started, keep Pupper supported and use the physical Stop.'};
  t.connection.textContent = 'Offline';
  if (setupTeam === t) $('setup-supported').checked = false;
  if (socket) socket.close();
}

async function beginSetup(t, action) {
  if (!t || t.online || t.connecting || t.setup.busy || state.phase !== 'idle') return;
  const request = {action};
  if (action === 'home') {
    if (!$('setup-supported').checked) return;
    request.supported = true;
    request.home_only = $('setup-home-mode').value === 'home-only';
  }
  const setup = {action, homeOnly:request.home_only, busy:true, message:'Connecting to the Pi…', discovered:t.setup.discovered};
  t.setup = setup;
  t.feedback.textContent = '';
  let completed = false;
  const fail = detail => {
    if (t.setup !== setup) return;
    setup.message = detail;
    setup.failed = true;
    setup.cue = null;
    setup.discovered = false;
    t.connection.textContent = 'Offline';
    render();
  };
  t.connection.textContent = 'Setting up…';
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
      if (event.stage === 'stand_prompt' && action === 'home' && !request.home_only) {
        setup.cue = event.cue;
        t.connection.textContent = 'Ready to stand';
      } else if (event.stage === 'complete') {
        completed = event.action === action;
        if (typeof event.match_capable === 'boolean') t.matchCapable = event.match_capable;
        setup.cue = null;
        setup.connectAfterHome = completed && t.matchCapable !== false && action === 'home' && !request.home_only && event.home_only === false;
        t.connection.textContent = action === 'prepare' ? 'Prepared' : 'Homed';
      } else if (event.stage === 'error') {
        fail(event.message || 'The Pi reported a setup error.');
      }
      render();
    };
    socket.onerror = () => fail('Could not reach the robot setup session.');
    socket.onclose = () => {
      if (t.setup !== setup) return;
      setup.busy = false;
      if (setupTeam === t && action === 'home') $('setup-supported').checked = false;
      setup.socket = null;
      setup.cue = null;
      if (!completed && !setup.failed) fail('Setup connection lost. Keep Pupper supported and check the Pi before retrying.');
      if (setup.connectAfterHome) {
        if (setupTeam === t) showSetup(null);
        void t.openRunner(true);
      }
      render();
    };
  } catch (e) {
    setup.busy = false;
    if (setupTeam === t && action === 'home') $('setup-supported').checked = false;
    fail(e.message);
  }
}

$('save-address').onclick = async () => {
  const t = setupTeam;
  if (!t || t.savingAddress || t.setup.busy || t.connecting || t.online || state.phase !== 'idle') return;
  t.savingAddress = true; render();
  const host = $('setup-address').value.trim();
  try {
    await post('discover', {slot:t.slot, host, save_address:true});
    if (host) localStorage.setItem(addressKey(t.slot), host);
    else localStorage.removeItem(addressKey(t.slot));
    t.setup = {message:host ? 'Saved SSH address: ' + host : 'Default connection restored.'};
    if (setupTeam === t) { $('setup-supported').checked = false; $('setup-address').value = host; }
  } catch (error) { t.setup.message = error.message; }
  finally { t.savingAddress = false; render(); }
};
$('setup-close').onclick = () => { const opener = setupTeam?.options; showSetup(null); opener?.focus(); };
$('setup-supported').onchange = renderSetup;
$('setup-home-mode').onchange = renderSetup;
$('setup-prepare').onclick = () => beginSetup(setupTeam, 'prepare');
$('setup-home').onclick = () => beginSetup(setupTeam, 'home');
$('setup-stand').onclick = () => {
  const setup = setupTeam?.setup;
  if (!setup?.cue || setup.socket?.readyState !== WebSocket.OPEN) return;
  setup.socket.send(JSON.stringify({action:'stand', cue:setup.cue}));
  setup.cue = null;
  setup.message = 'Standing cue sent. Waiting for the walker…';
  render();
};

function addTeam(slot, config) {
  const pane = $('team').content.firstElementChild.cloneNode(true);
  const el = key => pane.querySelector('.' + key);
  const t = {slot,id:config.robot_id,connect:el('connect'),modeEl:el('mode'),feedback:el('feedback'),
             ballMode:el('ball-mode'),ballReading:el('ball-reading'),setup:{message:''},options:el('options'),connection:el('connection'),online:false,connecting:false,ready:false,mode:null,requested:null,socket:null,stopping:false};
  teams[slot] = t;
  pane.classList.add(slot);
  el('team-label').textContent = slot === 'blue' ? 'HOME' : 'AWAY';
  pane.querySelector('h2').textContent = slot === 'blue' ? 'Blue Pupper' : 'Red Pupper';
  el('guide').textContent = KEY_GUIDE[slot];
  $('teams').appendChild(pane);
  const settings = el('team-settings'), options = el('options');
  const toggle = visible => {settings.hidden = !visible; options.setAttribute('aria-expanded',String(visible));};
  const title = pane.querySelector('h2').textContent;
  options.setAttribute('aria-label', 'Set up ' + title);
  options.setAttribute('aria-controls', 'robot-setup');
  options.onclick = () => {
    if (t.online) toggle(settings.hidden);
    else showSetup(setupTeam === t ? null : t);
  };
  el('disconnect').onclick = () => { clearKeys(slot); t.socket?.close(); };
  t.ballMode.onchange = () => { t.requestedBall = t.ballMode.value; t.ballMode.blur(); };
  t.modeEl.onchange = () => { t.requested = {mode:t.modeEl.value, expires:performance.now()+400}; clearKeys(slot); t.modeEl.blur(); };
  t.openRunner = async (alreadyDiscovered = false) => {
    if (t.online) { toggle(settings.hidden); return; }
    if (t.connecting || t.socket || t.setup.busy || t.savingAddress) return;
    if (setupTeam === t) showSetup(null);
    const attempt = Symbol();
    t.attempt=attempt; t.connecting=true; t.connect.disabled=true; t.stopping=false; t.feedback.textContent='';
    el('connection').textContent='Searching…'; message('');
    try {
      if (!alreadyDiscovered) await discoverTeam(slot);
      if(t.attempt!==attempt) return;
      const sock = new WebSocket('ws://' + location.host + '/play/' + slot + '?owner=' + owner);
      t.socket = sock; el('connection').textContent='Connecting…';
      sock.onmessage = ({data}) => {
        const event = JSON.parse(data);
        if (event.event === 'preparing') {
          t.connection.textContent = 'Preparing…';
          return;
        }
        if (event.event === 'profile' || event.event === 'home_required') {
          t.matchCapable = event.match_capable !== false;
          el('imu-profile').textContent = 'IMU profile · ' + (event.imu_recovery || 'strict').replaceAll('_', ' ');
          if (event.event === 'home_required') {
            t.needsHome = true;
            t.joystickListed = event.joystick_listed;
          }
          return;
        }
        if (event.event === 'onboard_ball' && event.robot_id === t.id && event.onboard_ball) {
          t.ball = event.onboard_ball;
          t.ballReceived = performance.now();
          if (!t.requestedBall && ['pink','confirmed','off'].includes(t.ball.mode)) t.ballMode.value = t.ball.mode;
          render();
          return;
        }
        if (event.event === 'stop_result') {
          t.feedback.textContent = event.confirmed ? 'Robot acknowledged Stop.' : 'Stop was not confirmed.';
          return;
        }
        if (event.event !== 'controls') {
          if ((event.event==='error'||event.accepted===false) && event.reason) t.feedback.textContent=event.reason;
          return;
        }
        if (t.stopping) return;
        t.imu = event.imu;
        if (t.imu?.state !== 'healthy') down.clear();
        if (t.mode !== event.mode || !event.ready) clearKeys(slot);
        t.online = true; t.connecting=false; t.ready=event.ready; t.mode=event.mode;
        if (pane.dataset.online !== 'true') {
          options.textContent = '⚙'; options.setAttribute('aria-label', title + ' settings');
          settings.id = slot + '-settings'; options.setAttribute('aria-controls', settings.id);
        }
        pane.dataset.online='true'; t.connect.disabled=false; options.disabled=false;
        el('connection').textContent=event.ready?'Ready':'Connected';
        t.connect.textContent='Connected'; t.modeEl.value=t.mode;
        el('guide').hidden=t.mode!=='teleop';
        let command=null;
        if (t.requestedBall) {
          command = {command:'onboard_ball_mode', mode:t.requestedBall};
          t.requestedBall = null;
        } else if (t.requested) {
          if (t.requested.expires>=performance.now()) command={command:'mode',mode:t.requested.mode};
          t.requested=null;
        } else if (t.mode==='teleop' && state.phase==='playing' && state.camera?.match?.epoch && !pending) {
          if (!state.camera.live || state.camera.match.phase!=='playing') clearKeys(slot);
          command={command:'drive',epoch:state.camera.match.epoch,...readDrive(down,slot)};
        }
        if (command && sock.readyState===WebSocket.OPEN && sock.bufferedAmount===0) {
          sock.send(JSON.stringify({...command,robot_id:t.id,ticket:event.ticket}));
        }
      };
      sock.onerror=()=>{ t.feedback.textContent='Could not open the prepared Pi session.'; };
      sock.onclose=()=>{
        t.socket=null;t.online=t.connecting=t.ready=false;t.requested=null;t.mode=null;
        t.ball = t.requestedBall = null;
        t.ballReading.textContent = 'Waiting for camera telemetry.';
        clearKeys(slot);pane.dataset.online='false';toggle(false);
        options.textContent='Set up'; options.setAttribute('aria-label', 'Set up ' + title);
        options.setAttribute('aria-controls', 'robot-setup'); t.setup = {message:''};
        el('connection').textContent='Offline';t.connect.textContent='Connect';
        if (t.needsHome) {
          t.needsHome = false;
          t.setup = {prepared:true, discovered:true, message:t.matchCapable ?
            'Connected. Support Pupper on the stand, then select Home.' :
            'Supported damping bench profile. Keep Pupper supported for the entire test.'};
          t.connection.textContent = 'Needs Home';
          t.connect.textContent = 'Home';
          showSetup(t);

        } else if (!t.feedback.textContent) t.feedback.textContent='Session closed. Check power, Wi-Fi and homing if unexpected.';
        render();
      };
    } catch(e) {
      if(t.attempt!==attempt) return;
      t.feedback.textContent=e.message; el('connection').textContent='Offline'; t.connecting=false; render();
    }
  };
  t.connect.onclick = () => t.setup.prepared ? showSetup(t) : t.openRunner();
}
function stopAll() {
  down.clear();
  for (const t of Object.values(teams)) {
    cancelSetup(t);
    t.requested=null;
    t.attempt=null;
    if(t.connecting && !t.socket){
      t.connecting=false; t.connect.disabled=false;
      t.connect.closest('.team').querySelector('.connection').textContent='Offline';
    }
    if (t.socket?.readyState===WebSocket.OPEN) {
      t.stopping=true; t.feedback.textContent='Stop requested…';
      t.socket.send(JSON.stringify({command:'stop',robot_id:t.id}));
    } else if (t.socket) t.socket.close();
  }
  void action('match/end');
}
$('stop-all').onclick=stopAll;
window.addEventListener('keydown',e=>{
  if (e.code==='Escape') {if(!e.repeat)stopAll();return;}
  const t=teams[slotForKey(e.code)];
  if (state.phase!=='playing'||!state.camera?.live||state.camera.match?.phase!=='playing'||!t?.ready||t.stopping||t.mode!=='teleop'||e.metaKey||e.ctrlKey||e.altKey||e.target?.matches('input,select,textarea,[contenteditable="true"]'))return;
  e.preventDefault(); if(!e.repeat)down.add(e.code);
});
window.addEventListener('keyup',e=>down.delete(e.code));
window.addEventListener('blur',()=>down.clear());
document.addEventListener('visibilitychange',()=>{if(document.hidden)down.clear();});
window.addEventListener('pagehide',()=>{down.clear();for(const t of Object.values(teams)){cancelSetup(t);t.socket?.close();}});
$('start-match').onclick=()=>action('match/start');
$('end-match').onclick=()=>{down.clear();return action('match/end');};
$('resume-balance').onclick=()=>{down.clear();return action('match/balance-resume',{id:state.balance?.id});};
$('resume-match').onclick=()=>action('match/resume');
function panel(open) {
  $('fieldpanel').hidden=!open; $('camera-settings').setAttribute('aria-expanded',String(open)); render();
}
$('camera-settings').onclick=()=>panel($('fieldpanel').hidden);
$('camera-connect').onclick=()=>{panel(true);if(!state.camera?.frame && state.phase==='idle')openCamera();};
function clearSnapshot(){
  frozen=null;survey=null;pickingFar=false;frameKey='';$('survey-help').textContent='';
}
async function takeSnapshot(){
  const shot = await post('camera/snapshot');
  frozen=shot;survey=null;pickingFar=false;$('view').src=shot.image;
  const fit = shot.metrics?.reference_error_px;
  const sensitivity = shot.metrics?.far_sensitivity_m;
  const info = ['Snapshot '+shot.seq, shot.status, shot.lens];
  if(Number.isFinite(fit)) info.push('reference error '+fit.toFixed(1)+' px');
  if(Number.isFinite(sensitivity)) info.push('far-corner sensitivity '+(sensitivity*100).toFixed(1)+' cm');
  if(shot.mode==='corner' && shot.fit_valid) info.push('Confirm only if each physical line intersection lies inside its labelled 7.5 cm ring and both end lines align.');
  if(shot.mode==='corner' && !shot.fit_valid && shot.ids.includes(state.camera_settings?.anchor?.tag_id)) info.push('Mark the diagonally opposite floor-line intersection to add a distant reference.');
  $('survey-help').textContent=info.join(' · ');
  render();
  return shot;
}
$('field-snapshot').onclick=async()=>{
  try{if(frozen){clearSnapshot();render();}else await takeSnapshot();}
  catch(e){message(e.message);}
};
$('far-corner').onclick=()=>{
  if(!frozen)return;
  const opposite={west_north:'southeast',east_north:'southwest',west_south:'northeast',east_south:'northwest'};
  pickingFar=true;survey=null;
  $('survey-help').textContent='Click the '+opposite[state.camera_settings.anchor.corner]+' line intersection on the floor. Use a distinct tape cross if the lines are unclear.';
};
$('field-confirm').onclick=async()=>{
  if(!frozen)return;
  await action('camera/confirm',{token:frozen.token});
  if(!lastError){clearSnapshot();render();}
};
function openCamera(){
  clearSnapshot();
  const field_mode=$('field-mode').value, anchor={corner:$('anchor-corner').value};
  if(field_mode==='corner') for(const [key,id] of Object.entries(anchorIds)){
    if($(id).value.trim()==='' || !Number.isFinite($(id).valueAsNumber)){message('Enter '+$(id).parentElement.textContent.trim()+'.');return;}
    anchor[key]=$(id).valueAsNumber;
  }
  const patch_height=$('patch-height').valueAsNumber;
  if($('robot-tracking').value==='colors' && !Number.isFinite(patch_height)){message('Enter the height of the patch surfaces above the floor.');return;}
  return action('camera',{source:$('camera-source').value, appearance:$('appearance').checked, patch_height,
    field_mode, robot_tracking:$('robot-tracking').value, anchor});
}
$('apply-camera').onclick=openCamera;
$('self-test').onclick = async () => {
  const button = $('self-test'), result = $('self-test-result');
  button.disabled = true;
  result.textContent = 'Checking the lobby with a simulated field…';
  try {
    const report = await post('self-test');
    result.textContent = report.ok ? report.summary : (report.summary || 'Offline check failed.');
  } catch (e) { result.textContent = e.message; }
  finally { button.disabled = false; }
};
$('view').onload=()=>{$('view').hidden=false;$('preview-wait').hidden=true;};
$('view').onerror=()=>{$('view').hidden=true;$('preview-wait').hidden=false;};
const postNames=['first west post','second west post','first east post','second east post'];
$('reset-goals').onclick=()=>action('goals/configured');
$('survey').onclick=async()=>{
  if(survey){clearSnapshot();render();return;}
  try{
    const shot=frozen || await takeSnapshot();
    survey={points:[],session:shot.source_session,revision:shot.revision,token:shot.token};
    $('survey-help').textContent='Click the base of the '+postNames[0]+' in the snapshot.';
  }catch(e){message(e.message);}
};
$('view').onclick=async e=>{
  if(!survey && !pickingFar)return;
  const r=e.currentTarget.getBoundingClientRect();
  if(pickingFar){
    pickingFar=false;
    await action('camera/far-corner',{token:frozen.token,point:[(e.clientX-r.left)/r.width,(e.clientY-r.top)/r.height]});
    if(!lastError){clearSnapshot();render();}
    return;
  }
  survey.points.push([(e.clientX-r.left)/r.width,(e.clientY-r.top)/r.height]);
  if(survey.points.length<4){$('survey-help').textContent='Click the base of the '+postNames[survey.points.length]+'.';return;}
  const data=survey;survey=null;
  await action('goals',data);
  if(!lastError)clearSnapshot();
  $('survey-help').textContent=lastError||'Goalposts saved for this field.';
};
async function refresh(){
  try {
    const response=await fetch('/api/state',{cache:'no-store'});
    if(!response.ok)throw new Error('The match server is unavailable');
    state=await response.json();render();
  } catch(e){message(e.message);}
  setTimeout(refresh,280);
}
try{
  const resp=await fetch('/slots');
  if(!resp.ok)throw new Error('The local match server is unavailable');
  for(const [slot,config] of Object.entries(await resp.json()))addTeam(slot,config);
  await refresh();
}catch(e){message(e.message);}
