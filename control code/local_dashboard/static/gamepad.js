export function setupGamepad(act, getState, client) {
  const $=id=>document.getElementById(id);
  let active=false, pending=false, selected=null, neutral=false;
  const deadzone=v=>Math.abs(v)<=.18?0:Math.sign(v)*Math.min(1,(Math.abs(v)-.18)/.82);
  const read=()=>{
    const pads=Array.from(navigator.getGamepads?.()||[]);
    return selected===null?pads.find(p=>p?.connected&&p.mapping==='standard'):pads[selected];
  };
  const values=p=>[p.axes[0],-p.axes[1],p.axes[2],-p.axes[3]].map(deadzone);
  async function stop() {
    active=false; neutral=false;
    await act('gamepad_stop');
  }
  $('gpstop').onclick=stop;
  $('gpstart').onclick=async()=>{
    const p=read();
    if(!p||p.mapping!=='standard'||p.axes.length<4||values(p).some(v=>v!==0)) {
      $('gpstatus').textContent='需要标准映射手柄，请先将两个摇杆回中'; return;
    }
    selected=p.index;
    pending=true;
    try { active=await act('gamepad_start'); neutral=active; }
    finally { pending=false; }
  };
  window.addEventListener('gamepaddisconnected',e=>{if(e.gamepad.index===selected){if(active)stop();selected=null;}});
  window.addEventListener('keydown',e=>{if(e.key==='Escape'){active=false;neutral=false;}});
  setInterval(async()=>{
    const p=read(), s=getState();
    $('gpstart').disabled=active||pending||!p||p.mapping!=='standard'||['running','stopping'].includes(s?.experiment?.status);
    if(pending)return;
    if(!active){$('gpstatus').textContent=p?'待连接':'等待手柄';return;}
    if(!p?.connected||p.mapping!=='standard'||p.axes.length<4||p.buttons[1]?.pressed||!s||s.owner!==client||!s.gamepad?.active){await stop();return;}
    const v=values(p), speed=Number($('groupspeed').value);
    if(!neutral||!Number.isFinite(speed)||speed<0||speed>1600){await stop();return;}
    $('gpstatus').textContent='已连接';
    pending=true;
    try { if(!await act('gamepad',{values:v,speed},true)){active=false;neutral=false;await act('gamepad_stop',{},true);} }
    finally {pending=false;}
  },100);
}
