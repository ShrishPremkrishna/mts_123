import {KEY_GUIDE, readDrive, slotForKey} from './drive.mjs';

const $ = id => document.getElementById(id);
const owner = crypto.randomUUID();
const teams = {}, down = new Set();
let state = {phase:'idle'}, pending = false, survey = null, frameKey = '', lastError = '';
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
  $('start-match').disabled = pending || state.phase !== 'idle' || !canStart;
  $('end-match').hidden = !['starting','playing'].includes(state.phase);
  $('resume-match').hidden = state.phase !== 'playing' || !match?.reset_ready;
  $('resume-match').disabled = pending || !state.resume_ready;
  $('survey').disabled = !cam?.live || !cam?.frame?.calibration_valid || state.phase !== 'idle';
  $('apply-camera').disabled = pending || state.phase !== 'idle';
  $('seen').textContent = cam?.frame ? [((visible.tags || []).filter(i => i>=0 && i<=5).length) + '/6 field tags',
    visible.ball ? 'Ball visible' : 'Searching for ball', ...Object.entries(visible.robots || {}).filter(([,v])=>v).map(([k])=>k==='striker'?'Blue visible':'Red visible')].join(' · ') : '';
  for (const t of Object.values(teams)) {
    t.modeEl.disabled = !t.online || state.phase !== 'idle';
    if (!t.online) t.connect.disabled = t.connecting || state.phase !== 'idle';
  }
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

function addTeam(slot, config) {
  const pane = $('team').content.firstElementChild.cloneNode(true);
  const el = key => pane.querySelector('.' + key);
  const t = {slot,id:config.robot_id,connect:el('connect'),modeEl:el('mode'),feedback:el('feedback'),
             online:false,connecting:false,ready:false,mode:null,requested:null,socket:null,stopping:false};
  teams[slot] = t;
  pane.classList.add(slot);
  el('team-label').textContent = slot === 'blue' ? 'HOME' : 'AWAY';
  pane.querySelector('h2').textContent = slot === 'blue' ? 'Blue Pupper' : 'Red Pupper';
  el('guide').textContent = KEY_GUIDE[slot];
  $('teams').appendChild(pane);
  const settings = el('team-settings'), options = el('options');
  const toggle = visible => {settings.hidden = !visible; options.setAttribute('aria-expanded',String(visible));};
  options.onclick = () => toggle(settings.hidden);
  el('disconnect').onclick = () => { clearKeys(slot); t.socket?.close(); };
  t.modeEl.onchange = () => { t.requested = {mode:t.modeEl.value, expires:performance.now()+400}; clearKeys(slot); t.modeEl.blur(); };
  t.connect.onclick = async () => {
    if (t.online) { toggle(settings.hidden); return; }
    if (t.connecting || t.socket) return;
    const attempt = Symbol();
    t.attempt=attempt; t.connecting=true; t.connect.disabled=true; t.stopping=false; t.feedback.textContent='';
    el('connection').textContent='Searching…'; message('');
    try {
      await post('discover',{slot});
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
        clearKeys(slot);pane.dataset.online='false';options.disabled=true;toggle(false);
        el('connection').textContent='Offline';t.connect.textContent='Connect';
        if (!t.feedback.textContent) t.feedback.textContent='Session closed. Check power, Wi-Fi and homing if unexpected.';
        render();
      };
    } catch(e) {
      if(t.attempt!==attempt) return;
      t.feedback.textContent=e.message; el('connection').textContent='Offline'; t.connecting=false; render();
    }
  };
}
function stopAll() {
  down.clear();
  for (const t of Object.values(teams)) {
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
window.addEventListener('pagehide',()=>{down.clear();for(const t of Object.values(teams))t.socket?.close();});
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
