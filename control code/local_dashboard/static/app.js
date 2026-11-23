import { setupGamepad } from './gamepad.js';
import { setupCamera } from './camera.js';
import { createScene } from './scene.js';
const $ = selector => document.querySelector(selector);
const colors = ['var(--ch1)', 'var(--ch2)', 'var(--ch3)'];
const client = crypto.randomUUID();
let page = 'overview';
let state = null, selected = 0, port = '', busy = false, deviceSignature = '';
const escapeHtml = value => String(value ?? '').replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
function toast(message) {
  $('#toast').textContent = message;
  $('#toast').style.display = 'block';
  clearTimeout(toast.timer);
  toast.timer = setTimeout(() => $('#toast').style.display = 'none', 4500);
}
async function act(action, data = {}, quiet = false) {
  try {
    const response = await fetch('/api/action', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({action,client,...data})});
    const result = await response.json();
    if (!response.ok) throw Error(result.error);
    if (!quiet) await refresh();
    return true;
  } catch (error) {
    if (!quiet) toast(error.message);
    return false;
  }
}
function statusText(axis) { return axis.error ? (axis.online&&axis.comm_failures?'通信恢复中':'故障') : !axis.online ? '未接入' : axis.moving ? '运动中' : axis.armed ? '已启用' : '已连接'; }
function statusClass(axis) { return axis.error ? 'error' : !axis.online ? '' : axis.moving ? 'busy' : 'ready'; }
function selectAxis(i) { selected=i; render(); }
const cameraPanel=setupCamera(act,()=>{
  showPage('overview');
  $('#pretestopen').closest('.consolepanel').scrollIntoView({block:'start'});
});
setupGamepad(act,()=>state,client);
$('#axes').innerHTML = Array.from({length:3}, (_,i) => `
  <article class="axis" id="axis${i}" style="--channel:${colors[i]}">
    <div class="axishead"><h3><span class="channelnum">0${i+1}</span> 电缸 ${i+1}<span class="deviceid" id="id${i}"></span></h3><span class="status" id="status${i}">未接入</span></div>
    <div class="readout"><b id="pos${i}">—</b><span>mm</span><span class="poslabel">反馈位置</span></div>
    <div class="positionsub"><span>目标<b id="target${i}">0.000</b></span><span id="age${i}">无有效反馈</span></div>
    <div class="track"><span id="track${i}" style="width:0"></span></div><div class="limitlabels"><span id="low${i}">0</span><span id="high${i}">16 mm</span></div>
    <div class="axismeta"><span>速度 <b id="speed${i}">0.5 mm/s</b></span><span>温度 <b id="temp${i}">—</b></span></div>
    <div class="axisbuttons"><button id="connect${i}">接入设备</button><button id="arm${i}">启用</button><button id="stop${i}">■ 停止</button></div>
    <div class="targetrow"><label class="targetinput"><span class="sr">电缸 ${i+1} 绝对目标</span><input id="input${i}" aria-label="电缸 ${i+1} 绝对目标" type="number" min="0" max="16" step="0.1" value="0"><span>mm</span></label><button class="primary" id="move${i}">移动 →</button></div>
    <div class="jog"><button id="minus${i}" aria-label="电缸 ${i+1} 减少 0.1 mm">− 0.1 mm</button><button id="plus${i}" aria-label="电缸 ${i+1} 增加 0.1 mm">+ 0.1 mm</button></div><div class="axiserror" id="error${i}"></div>
  </article>`).join('');
$('#axistree').innerHTML = Array.from({length:3},(_,i)=>`<button class="axisnav" id="axisnav${i}"><i class="dot" id="navdot${i}"></i><span>通道 0${i+1}</span><small id="navstatus${i}">未接入</small></button>`).join('');
const model = createScene(i => {
  selectAxis(i);
  if (page !== 'hydraulic') showPage('hydraulic');
});
for (let i=0;i<3;i++) {
  $('#axis'+i).addEventListener('click',()=>selectAxis(i));
  $('#axisnav'+i).onclick=()=>{selectAxis(i);showPage('hydraulic');};
  $('#connect'+i).onclick=()=>act(state?.axes[i].online?'disconnect_axis':'connect_axis',{axis:i,port});
  $('#arm'+i).onclick=()=>act('arm',{axis:i});
  $('#stop'+i).onclick=()=>act('stop_axis',{axis:i});
  $('#move'+i).onclick=()=>{
    const input=$('#input'+i);
    if (!input.value.trim() || !Number.isFinite(input.valueAsNumber)) return toast('请输入有效的目标位置');
    act('move',{axis:i,value:input.valueAsNumber});
  };
  $('#minus'+i).onclick=()=>act('jog',{axis:i,value:-.1});
  $('#plus'+i).onclick=()=>act('jog',{axis:i,value:.1});
}
for (const b of document.querySelectorAll('[data-view]')) b.onclick=()=>model.view(b.dataset.view);
$('#resetview').onclick=()=>model.view('overview');
function showPage(name){
  if (page !== name && state?.owner===client && state.propulsion.targets.some(Boolean)) endProp();
  page=name;
  window.scrollTo({top:0,behavior:'instant'});
  for(const [id,key] of [['hydraulicpage','hydraulic'],['propsection','prop'],['camerasection','camera'],['overviewcards','overview']])$('#'+id).hidden=key!==name;
  for(const key of ['overview','hydraulic','prop','camera'])$('#nav'+key).classList.toggle('selected',key===name);
  $('.viewportpanel').hidden=name==='camera';
  $('#pagetitle').textContent={overview:'系统总览',hydraulic:'液压驱动',prop:'滚轮推进器',camera:'深度视觉'}[name];
  document.body.dataset.page=name;
  $('.channels').hidden=name==='prop';
  $('#selectedmodel').textContent=name==='prop'?'M1–M4 / 板端状态':`通道 0${selected+1} / ID ${state?.axes[selected].id??selected+1}`;
  model.view(name==='hydraulic'?'hydraulic':name==='prop'?'advancer':'overview');
  document.body.classList.remove('devicesopen');$('#devicepanel').setAttribute('aria-expanded','false');
}
for(const key of ['overview','hydraulic','prop','camera'])$('#nav'+key).onclick=()=>showPage(key);
document.addEventListener('componentselect',e=>showPage(e.detail));
$('#motorcards').innerHTML=Array.from({length:4},(_,i)=>`<article class="motorcard"><header>M${i+1}<span id="mstate${i}">无反馈</span></header><strong id="mspeed${i}">—</strong><small>step/s · 板端脉冲频率</small><p>电流设定 <b id="mcurrent${i}">—</b> mA</p><label class="motorspeed">速度 <input id="speedmotor${i}" type="number" value="400" min="0" max="1600" step="1" aria-label="M${i+1} 速度"> step/s</label><button id="applymotor${i}">应用速度</button><div class="motorbuttons"><button id="reverse${i}">后退</button><button id="halt${i}">停止</button><button id="forward${i}">前进</button></div></article>`).join('');
function speedValue(id){const input=$('#'+id),v=input.valueAsNumber;if(!input.value.trim()||!Number.isInteger(v)||v<0||v>1600){toast('速度须为 0–1600 的整数');return null;}return v;}
for(let i=0;i<4;i++){
  for(const [name,sign] of [['forward',1],['reverse',-1]])$('#'+name+i).onclick=()=>{const v=speedValue('speedmotor'+i);if(v!==null)act('motor',{motor:i,value:sign*v});};
  $('#halt'+i).onclick=()=>act('motor_stop',{motor:i});
  $('#applymotor'+i).onclick=()=>{const v=speedValue('speedmotor'+i),t=state?.propulsion.targets[i]||0;if(v!==null&&t)act('motor',{motor:i,value:Math.sign(t)*v});else if(v!==null)toast('速度已设置，选择前进或后退启动');};
}
$('#overviewcards').innerHTML=[['hydraulic','01','液压驱动','三通道独立位移'],['prop','02','滚轮推进器','四电机状态'],['camera','03','深度视觉','RealSense D405']].map(([key,num,title,sub])=>`<button class="summarycard" data-page="${key}"><span>${num} / ${sub}</span><h3>${title} →</h3><p id="summary${key}">未接入</p></button>`).join('');
for(const button of document.querySelectorAll('[data-page]'))button.onclick=()=>showPage(button.dataset.page);
async function scan() {
  $('#scan').disabled=true;$('#scan').textContent='…';
  await act('scan');$('#scan').textContent='↻';$('#scan').disabled=false;
}

$('#scan').onclick=scan;
$('#stop').onclick=()=>act('stop_all');
$('#armgroup').onclick=()=>act('arm_group');
$('#groupmove').onclick=()=>{
  const inputs=[0,1,2].map(i=>$('#g'+i));
  if(inputs.some(e=>!e.value.trim()||!Number.isFinite(e.valueAsNumber)))return toast('请完整填写三个目标位置');
  act('move_group',{values:inputs.map(e=>e.valueAsNumber)});
};
$('#pconnect').onclick=()=>act(state?.propulsion.online?'disconnect_prop':'connect_prop',{port});
$('#pstop').onclick=()=>act('stop_prop');
function endProp(){act('stop_prop',{},true);}
for(const [id,sign] of [['pback',-1],['pforward',1]])$('#'+id).onclick=()=>{const v=speedValue('groupspeed');if(v!==null)act('propulsion',{value:sign*v});};
$('#applygroup').onclick=()=>{const v=speedValue('groupspeed'),targets=state?.propulsion.targets||[];if(v===null)return;if(targets.length&&targets.every(t=>t===targets[0])&&targets[0])act('propulsion',{value:Math.sign(targets[0])*v});else toast('速度已设置，选择一起前进或后退启动');};
const connectionWorker=new Worker('/connection-worker.js');
connectionWorker.postMessage({client,renewMotors:false});
connectionWorker.onmessage=({data})=>{$('#heartbeatstatus').textContent=!data.ok?'心跳失败：'+data.error:data.accepted?'控制心跳正常':data.owner?'其他页面持有控制权':'服务已连接 · 尚未取得控制权';};
connectionWorker.onerror=e=>{$('#heartbeatstatus').textContent='心跳线程错误：'+e.message;};
window.addEventListener('keydown',e=>{if(e.key==='Escape')act('stop_all');});

function renderDevices(){
  const signature=JSON.stringify(state.devices);
  if(signature===deviceSignature)return;
  deviceSignature=signature;
  if(port&&!state.devices.some(d=>d.port===port))port='';
  $('#devices').innerHTML=state.devices.length?
    `<div class="device"><label><input type="radio" name="port" value="" ${!port?'checked':''}>自动匹配 · 唯一设备</label></div>`+
    state.devices.map(d=>`<div class="device"><label><input type="radio" name="port" value="${escapeHtml(d.port)}" ${port===d.port?'checked':''}>${escapeHtml(d.port.replace('/dev/cu.',''))}</label><p>${escapeHtml(d.kind)} ${d.ids.length?'· ID '+d.ids.join(' / '):''} ${escapeHtml(d.error||'')}</p></div>`).join(''):'<p class="empty">尚未扫描设备</p>';
  document.querySelectorAll('input[name=port]').forEach(e=>e.onchange=()=>port=e.value);
}
function render(){
  if(!state)return;
  connectionWorker.postMessage({client,renewMotors:state.owner===client&&!['running','stopping'].includes(state.experiment?.status)&&state.propulsion.targets.some(Boolean)});

  $('#selectedmodel').textContent=page==='prop'?'M1–M4 / 板端状态':`通道 0${selected+1} / ID ${state.axes[selected].id}`;
  $('#notice').textContent='扫描后接入设备；重连不恢复运动。';
  $('#controlreason').textContent=state.stop_reason?'最近停机：'+state.stop_reason:'';
  $('#controlreason').hidden=!state.stop_reason;
  $('#count').textContent=`${state.axes.filter(a=>a.online).length+(state.propulsion.online?1:0)+(state.camera.online?1:0)} / 5`;
  for(let i=0;i<3;i++){
    const a=state.axes[i];
    $('#axis'+i).classList.toggle('selected',selected===i);$('#axisnav'+i).classList.toggle('selected',selected===i);
    $('#id'+i).textContent='ID '+a.id;$('#status'+i).textContent=statusText(a);$('#status'+i).className='status '+statusClass(a);
    $('#navstatus'+i).textContent=statusText(a);$('#navdot'+i).className='dot '+(a.error?'fault':a.moving?'moving':a.online?'online':'');
    $('#pos'+i).textContent=a.online?a.position.toFixed(3):'—';$('#target'+i).textContent=a.target.toFixed(3);
    $('#age'+i).textContent=a.online&&a.age!==null?`反馈 ${a.age.toFixed(1)} s 前`:'无有效反馈';
    $('#low'+i).textContent=a.minimum;$('#high'+i).textContent=a.maximum+' mm';$('#speed'+i).textContent=a.speed+' mm/s';$('#temp'+i).textContent=a.online?`${a.temperature??'—'} °C`:'—';
    $('#track'+i).style.width=Math.max(0,Math.min(100,(a.position-a.minimum)/(a.maximum-a.minimum)*100))+'%';
    $('#connect'+i).disabled=false;$('#connect'+i).textContent=a.online?'断开设备':'接入设备';$('#arm'+i).textContent=a.armed?'已启用':'启用';$('#arm'+i).disabled=!a.online||a.armed||!!a.error;
    for(const s of ['move','minus','plus'])$('#'+s+i).disabled=!a.online||!a.armed||!!a.error;
    $('#stop'+i).disabled=!a.online;$('#input'+i).min=$('#g'+i).min=a.minimum;$('#input'+i).max=$('#g'+i).max=a.maximum;$('#error'+i).textContent=a.error;
  }
  $('#armgroup').disabled=!state.axes.every(a=>a.online&&!a.error&&a.age!==null&&a.age<1)||state.axes.every(a=>a.armed);
  $('#armgroup').textContent=state.axes.every(a=>a.armed)?'三轴已启用':'一键启用三轴';
  $('#groupmove').disabled=!state.axes.every(a=>a.online&&a.armed&&!a.error);
  const p=state.propulsion;$('#pstate').textContent=p.error?'故障':p.online?'已连接 · '+p.backend:'未接入';$('#pstate').className='status '+(p.error?'error':p.online?'ready':'');
  $('#navpropstate').textContent=p.error?'故障':p.online?'已连接':'未接入';$('#propdot').className='dot '+(p.error?'fault':p.online?'online':'');
  $('#pconnect').textContent=p.online?'断开推进器':'接入推进器';$('#pspeed').textContent=p.speed===null?'独立':p.speed;$('#pback').disabled=$('#pforward').disabled=!p.online||!!p.error;
  const fresh=p.online&&!p.error&&p.telemetry_age!==null&&p.telemetry_age<2;
  for(let i=0;i<4;i++){
    const m=fresh?p.motors?.[i]:null;
    $('#mstate'+i).textContent=p.error?'板端故障':!p.online?'未接入':p.backend==='nano'?'协议不支持':m?'反馈在线':'等待反馈';
    for(const name of ['forward','reverse','halt','applymotor'])$('#'+name+i).disabled=!p.online||!!p.error||!['pcb','sim'].includes(p.backend)||['running','stopping'].includes(state.experiment?.status);
    $('#mspeed'+i).textContent=m?m.speed:'—';$('#mcurrent'+i).textContent=m?m.current_ma:'—';
  }
  $('#summaryhydraulic').textContent=state.axes.map(a=>`ID ${a.id}: ${a.online&&!a.error?a.position.toFixed(3)+' mm':statusText(a)}`).join(' · ');
  $('#summaryprop').textContent=p.error?'板端故障':p.online?`${p.backend.toUpperCase()} · ${fresh?'四路遥测在线':'暂无四路遥测'}`:'未接入';
  cameraPanel.render(state);
  renderDevices();
  renderConsole();
  if(['running','stopping'].includes(state.experiment?.status))for(const b of document.querySelectorAll('.axes button,#groupmove,#armgroup,#pforward,#pback,#applygroup,#pconnect,#scan'))b.disabled=true;
  else {$('#scan').disabled=false;$('#pconnect').disabled=false;$('#applygroup').disabled=!p.online;}

  model.update(state,selected);
}
async function refresh(){
  if(busy)return;busy=true;
  try{
    const response=await fetch('/api/state',{signal:AbortSignal.timeout(3000)});if(!response.ok)throw Error('HTTP '+response.status);state=await response.json();
    $('#connection').innerHTML='<i></i>'+(state.owner&&state.owner!==client?'其他页面正在控制':'本地服务在线');$('#serverdot').className='dot online';render();
  }catch{
    state=null;cameraPanel.render(null);connectionWorker.postMessage({client,renewMotors:false});model.update(null,selected);for(let i=0;i<4;i++){$('#mstate'+i).textContent='反馈中断';$('#mspeed'+i).textContent=$('#mcurrent'+i).textContent='—';}for(let i=0;i<3;i++){$('#pos'+i).textContent='—';}$('#pstate').textContent='反馈中断';$('#connection').textContent='服务离线';$('#serverdot').className='dot fault';
    for(let i=0;i<3;i++){$('#status'+i).textContent='反馈中断';$('#status'+i).className='status error';$('#navdot'+i).className='dot fault';}
    for(const button of document.querySelectorAll('.axes button,.propcontrols button,.motorcard button,#applygroup,#runcommand,#armgroup,#groupmove,#pretestopen,#pretestpause,#pretestresume'))button.disabled=true;
  }finally{busy=false;}
}

setInterval(refresh,300);refresh();

$('#devicepanel').onclick=()=>{const open=document.body.classList.toggle('devicesopen');$('#devicepanel').setAttribute('aria-expanded',String(open));};

showPage(window.location.hash==='#camera'?'camera':'overview');

let localOutput='';
function renderConsole(){
  const e=state?.experiment;if(!e)return;
  const names={idle:'空闲',running:'运行中',stopping:'停止中',completed:'已完成',failed:'执行失败',stopped:'已停止'};
  $('#scriptstate').textContent=names[e.status]||e.status;
  $('#runcommand').disabled=['running','stopping'].includes(e.status);
  $('#pretestopen').disabled=$('#runcommand').disabled;
  const pretest=e.pretest, active=e.status==='running'&&!!pretest;
  $('#pretestpause').hidden=!active||pretest.phase==='paused';
  $('#pretestpause').disabled=e.pause_requested;
  $('#pretestpause').textContent=e.pause_requested?'将在本轮结束后暂停':'本轮结束后暂停';
  $('#pretestresume').hidden=!active||pretest.phase!=='paused';
  $('#pretestresume').disabled=!e.pause_requested;
  $('#pretestprogress').hidden=!pretest;
  if(pretest){
    const phases={baseline:'采集静止参考',ready:'参考姿态已记录',prepare:'记录起始姿态',advance:'推进中',hold:'目标位停留',return:'回参考位并等待稳定',export:'保存视频',round_complete:'本轮已保存',paused:'已暂停',done:'采集完成'};
    const bar=$('#pretestbar');bar.max=pretest.total??27;bar.value=pretest.completed??0;
    const round=pretest.point?` · 第 ${pretest.point} 点 (${pretest.offset_mm.join(', ')} mm) · 第 ${pretest.repeat} 轮`:'';
    $('#pretestphase').textContent=`${active?(phases[pretest.phase]??'启动中'):names[e.status]} · 已保存 ${bar.value}/${bar.max}${round}`;
    const shape=state.camera.measurement?.shape_3d;
    const fresh=active&&state.camera.online&&state.camera.frame_age!==null&&state.camera.frame_age<1;
    const angle=fresh&&shape?.valid?shape.angle_deg:null;
    const peak=pretest.last_round?` · 上一轮 T1 ${pretest.last_peak_deg===null?'无有效三维角':pretest.last_peak_deg.toFixed(2)+'°'}${pretest.last_peak_status==='indistinguishable'?'（未超出参考抖动）':''}`:'';
    $('#pretestangle').textContent=`实时三维弯曲角 ${angle===null?'—':angle.toFixed(2)+'°'}${peak}`;
    $('#pretestfiles').textContent=pretest.directory??'';
  }
  const text=localOutput+(e.command?'\n$ '+e.command+'\n':'')+e.output;
  if($('#consoleoutput').textContent!==text){$('#consoleoutput').textContent=text||'输入 help 查看指令。';$('#consoleoutput').scrollTop=$('#consoleoutput').scrollHeight;}
  $('#commands').innerHTML=['help','scripts',...e.scripts.map(s=>'python '+s),'stop','clear'].map(s=>`<option value="${escapeHtml(s)}">`).join('');
}
$('#pretestopen').onclick=()=>{cameraPanel.beginPretest();showPage('camera');};
$('#pretestpause').onclick=()=>act('pretest_pause');
$('#pretestresume').onclick=()=>act('pretest_resume');
$('#consoleform').onsubmit=async e=>{e.preventDefault();const c=$('#command').value.trim();if(!c)return;
  if(c==='help')localOutput='help — 指令帮助\nscripts — 列出本机实验脚本\npython scripts/name.py [参数] — 运行本机 Python\nstop — 中止实验并停止设备\nclear — 清空控制台输出\n\npython scripts/status.py\npython scripts/camera_check.py\npython scripts/camera_record.py --seconds 30 --label reference\npython scripts/sample_positions.py --help\npython scripts/motor_run.py --motor 1 --speed 400 --seconds 2\n\n脚本通过 experiment_api.Robot 控制已接入设备；脚本是具有本机权限的 Python 程序，不是沙箱。';
  else if(c==='scripts')localOutput=(state?.experiment.scripts||[]).join('\n');
  else if(c==='clear'){localOutput='';await act('script_clear');}
  else if(c==='stop')await act('script_stop');
  else await act('script_run',{command:c});
  renderConsole();
};
$('#stopscript').onclick=()=>act('script_stop');
$('#command').oninput=()=>{$('#commandhint').textContent=$('#command').value.includes('motor_run')?'--motor 1–4 · --speed -1600…1600 · --seconds 0–60；会驱动已接入设备':'help · scripts · python scripts/name.py [参数] · stop · clear';};
