import {KEY_GUIDE, readDrive, slotForKey} from './drive.mjs';

const $ = id => document.getElementById(id);
const owner = crypto.randomUUID();
const teams = {}, down = new Set();
let state = {phase:'idle'}, pending = false, survey = null, frameKey = '', lastError = '';
let setupTeam = null;
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
  $('score-blue').textContent = match?.score?.['pupper-1'] ?? 0;
  $('score-red').textContent = match?.score?.['pupper-2'] ?? 0;
  $('camera-status').textContent = cam ? ((state.source === 'sim' ? 'Simulated · ' : '') + cam.status) : 'Disconnected';
  $('camera-connect').textContent = cam?.frame ? 'View field' : 'Connect camera';
  $('start-match').hidden = ['playing','stopping'].includes(state.phase);
  const visible = cam?.visible || {}, canStart = cam?.live && cam?.frame?.calibration_valid && visible.ball &&
        live.length && live.every(t => visible.robots?.[t.slot === 'blue' ? 'striker' : 'keeper']);
  $('start-match').disabled = pending || state.phase !== 'idle' || !canStart ||
    Object.values(teams).some(t => t.setup.busy);
  $('end-match').hidden = !['starting','playing'].includes(state.phase);
  $('resume-match').hidden = state.phase !== 'playing' || !match?.reset_ready;
  $('resume-match').disabled = pending || !state.resume_ready;
  $('survey').disabled = !cam?.live || !cam?.frame?.calibration_valid || state.phase !== 'idle';
  $('apply-camera').disabled = pending || state.phase !== 'idle';
  $('seen').textContent = cam?.frame ? [((visible.tags || []).filter(i => i>=0 && i<=5).length) + '/6 field tags',
    visible.ball ? 'Ball visible' : 'Searching for ball', ...Object.entries(visible.robots || {}).filter(([,v])=>v).map(([k])=>k==='striker'?'Blue visible':'Red visible')].join(' · ') : '';
  for (const t of Object.values(teams)) {
    t.modeEl.disabled = !t.online || state.phase !== 'idle';
    if (!t.online) {
      t.connect.disabled = t.connecting || t.setup.busy || state.phase !== 'idle';
      t.options.disabled = t.connecting || (state.phase !== 'idle' && !t.setup.busy);
      t.options.setAttribute('aria-expanded', String(setupTeam === t));
    }
  }
  renderSetup();
  if (!lastError) $('error').textContent = state.phase === 'playing' && match?.phase === 'goal' ?
    (match.reset_ready ? (state.resume_ready ? 'Ball in place. Ready to resume.' : 'Ball in place. Waiting for robots to reach home.') : 'Goal! Return the ball to the field.') : (state.message ||
    (state.source === 'sim' && cam ? 'Field preview is running. Select a live camera to start a match.' :
      canStart ? 'Field ready for kickoff.' : live.length && cam?.live ? 'Place the ball and connected teams in camera view.' : 'Connect a team and the field camera to play.'));
  if (!$('fieldpanel').hidden && cam?.frame) {
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

function showSetup(t) {
  if (setupTeam !== t) {
    $('setup-supported').checked = false;
    $('setup-home-mode').value = 'stand';
  }
  setupTeam = t;
  $('robot-setup').hidden = !t;
  render();
}

function renderSetup() {
  const t = setupTeam;
  if (!t) return;
  const setup = t.setup;
  const locked = setup.busy || state.phase !== 'idle';
  if (setup.busy && setup.action === 'home') {
    $('setup-supported').checked = true;
    $('setup-home-mode').value = setup.homeOnly ? 'home-only' : 'stand';
  }
  $('robot-setup').dataset.team = t.slot;
  $('setup-title').textContent = (t.slot === 'blue' ? 'Blue' : 'Red') + ' Pupper setup';
  $('setup-prepare').disabled = locked;
  $('setup-supported').disabled = locked;
  $('setup-home-mode').disabled = locked;
  $('setup-home').hidden = Boolean(setup.cue);
  $('setup-home').disabled = locked || !$('setup-supported').checked;
  $('setup-stand').hidden = !setup.cue;
  $('setup-stand').disabled = !setup.cue;
  if (setup.busy && setup.action === 'home') {
    $('setup-hint').textContent = 'Keep Pupper supported. Use the gamepad for physical Stop. Esc cancels a pending standing cue.';
  } else {
    $('setup-hint').textContent = $('setup-home-mode').value === 'home-only'
      ? 'Home the legs and leave the walker inactive.'
      : 'Standing requires a connected Stop gamepad and a fresh cue after homing.';
  }
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
    if (!setup.discovered) await post('discover', {slot:t.slot});
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
        setup.cue = null;
        setup.connectAfterHome = completed && action === 'home' && !request.home_only && event.home_only === false;
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
             setup:{message:''},options:el('options'),connection:el('connection'),online:false,connecting:false,ready:false,mode:null,requested:null,socket:null,stopping:false};
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
  t.modeEl.onchange = () => { t.requested = {mode:t.modeEl.value, expires:performance.now()+400}; clearKeys(slot); t.modeEl.blur(); };
  t.openRunner = async (alreadyDiscovered = false) => {
    if (t.online) { toggle(settings.hidden); return; }
    if (t.connecting || t.socket || t.setup.busy) return;
    if (setupTeam === t) showSetup(null);
    const attempt = Symbol();
    t.attempt=attempt; t.connecting=true; t.connect.disabled=true; t.stopping=false; t.feedback.textContent='';
    el('connection').textContent='Searching…'; message('');
    try {
      if (!alreadyDiscovered) await post('discover',{slot});
      if(t.attempt!==attempt) return;
      const sock = new WebSocket('ws://' + location.host + '/play/' + slot + '?owner=' + owner);
      t.socket = sock; el('connection').textContent='Connecting…';
      sock.onmessage = ({data}) => {
        const event = JSON.parse(data);
        if (event.event === 'stop_result') {
          t.feedback.textContent = event.confirmed ? 'Robot acknowledged Stop.' : 'Stop was not confirmed.';
          return;
        }
        if (event.event !== 'controls') {
          if ((event.event==='error'||event.accepted===false) && event.reason) t.feedback.textContent=event.reason;
          return;
        }
        if (t.stopping) return;
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
        if (t.requested) {
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
        clearKeys(slot);pane.dataset.online='false';toggle(false);
        options.textContent='Set up'; options.setAttribute('aria-label', 'Set up ' + title);
        options.setAttribute('aria-controls', 'robot-setup'); t.setup = {message:''};
        el('connection').textContent='Offline';t.connect.textContent='Connect';
        if (!t.feedback.textContent) t.feedback.textContent='Session closed. Check power, Wi-Fi and homing if unexpected.';
        render();
      };
    } catch(e) {
      if(t.attempt!==attempt) return;
      t.feedback.textContent=e.message; el('connection').textContent='Offline'; t.connecting=false; render();
    }
  };
  t.connect.onclick = () => t.openRunner();
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
$('resume-match').onclick=()=>action('match/resume');
function panel(open) {
  $('fieldpanel').hidden=!open; $('camera-settings').setAttribute('aria-expanded',String(open)); render();
}
$('camera-settings').onclick=()=>panel($('fieldpanel').hidden);
$('camera-connect').onclick=()=>{panel(true);if(!state.camera?.frame && state.phase==='idle')openCamera();};
function openCamera(){
  survey=null;frameKey='';
  return action('camera',{source:$('camera-source').value,appearance:$('appearance').checked});
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
$('survey').onclick=()=>{
  const cam=state.camera;
  if(survey){survey=null;$('survey-help').textContent='Survey canceled.';return;}
  if(!cam?.frame)return;
  survey={points:[],session:cam.source_session,revision:cam.frame.calibration_revision};
  $('survey-help').textContent='Click the base of the '+postNames[0]+'.';
};
$('view').onclick=async e=>{
  if(!survey)return;
  const r=e.currentTarget.getBoundingClientRect();
  survey.points.push([(e.clientX-r.left)/r.width,(e.clientY-r.top)/r.height]);
  if(survey.points.length<4){$('survey-help').textContent='Click the base of the '+postNames[survey.points.length]+'.';return;}
  const data=survey;survey=null;
  await action('goals',data);
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
