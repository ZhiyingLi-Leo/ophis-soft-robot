import { installRoiDrag } from './camera.js';

const $=id=>document.getElementById(id);

export function setupSideCamera(act){
  const key='camera.side_view';
  const device=(operation,args={})=>act('device',{device:key,operation,arguments:args});
  const image=$('side-preview'), select=$('side-devices');
  const roiFields=['side-x0','side-y0','side-x1','side-y1'];
  const colorFields=['side-white','side-sat','side-min-h','side-min-s','side-min-v','side-max-h','side-max-s','side-max-v'];
  const number=id=>$(id).valueAsNumber;
  let camera, fresh=false, locked=false, pending=false, listing=false;
  let lastFrame=null, previewPending=false, imageUrl=null, previousSettings='';
  let error='';

  function syncSettings(force=false){
    if(!camera)return;
    const s=camera.settings, signature=JSON.stringify(s);
    if(!force&&signature===previousSettings)return;
    previousSettings=signature;
    s.roi.forEach((value,i)=>$(roiFields[i]).value=+(value*100).toFixed(4));
    [s.white_min,s.white_sat,...s.orange_min,...s.orange_max].forEach((value,i)=>$(colorFields[i]).value=value);
  }
  function activeConnection(){
    return camera?.online||['连接中','等待侧视首帧'].includes(camera?.status);
  }
  function controls(){
    $('side-config').disabled=!camera||locked||pending;
    $('side-locate').disabled=!fresh;
    $('side-lock').hidden=!locked;
    $('side-discover').disabled=!camera||locked||listing;
    select.disabled=!camera||locked||listing||activeConnection();
    $('side-connect').disabled=!camera||locked||pending||(!activeConnection()&&!select.value);
    $('side-connect').textContent=activeConnection()?'断开侧视相机':'接入侧视相机';
    image.classList.toggle('drawing-enabled',fresh&&!locked&&!pending);
    $('side-error').textContent=error||camera?.error||camera?.record_error||'';
  }
  async function command(operation,args={}){
    pending=true;error='';controls();
    try{
      return await device(operation,args);
    }finally{pending=false;controls();}
  }
  $('side-discover').onclick=async()=>{
    listing=true;error='';controls();
    try{
      const response=await fetch('/api/camera/side-view/devices',{signal:AbortSignal.timeout(5000)});
      const devices=await response.json();
      if(!response.ok)throw Error(devices.error);
      const selected=camera?.device_id||select.value;
      select.replaceChildren(new Option('请选择摄像头',''));
      for(const {id,name,type} of devices){
        const label={external:'外接',built_in:'内置',continuity:'连续互通'}[type]||type;
        select.add(new Option(`${name} · ${label}`,id));
      }
      if(devices.some(d=>d.id===selected))select.value=selected;
      const ace=devices.filter(d=>d.type==='external'&&/insta360.*ace/i.test(d.name));
      if(!select.value&&ace.length===1)select.value=ace[0].id;
      if(!devices.length)error='未找到摄像头。请确认 USB 已连接并选择 Webcam 模式。';
    }catch(e){error=e.message;}finally{listing=false;controls();}
  };
  select.onchange=controls;
  $('side-connect').onclick=()=>activeConnection()?command('disconnect'):command('connect',{device_id:select.value});
  $('side-apply').onclick=()=>{
    const values=colorFields.map(number);
    const roi=roiFields.map(id=>number(id)/100);
    if(![...values,...roi].every(Number.isFinite)){error='请填写全部 ROI 和颜色参数。';controls();return;}
    command('configure',{roi,white_min:values[0],white_sat:values[1],
      orange_min:values.slice(2,5),orange_max:values.slice(5,8)});
  };
  $('side-locate').onclick=async()=>{if(await command('auto_locate'))syncSettings(true);};

  installRoiDrag(image,$('side-cropbox'),{
    enabled:()=>fresh&&!locked&&!pending,
    onSelection:async roi=>{
      if(!roi){error='区域宽高至少占画面的 5%。';controls();return;}
      if(await command('configure',{roi}))syncSettings(true);
    }
  });

  function clearPreview(){
    if(imageUrl)URL.revokeObjectURL(imageUrl);
    imageUrl=null;lastFrame=null;
    image.removeAttribute('src');image.hidden=true;$('side-placeholder').hidden=false;
  }
  async function preview(){
    if(previewPending||camera.frame_number===lastFrame||document.body.dataset.page!=='camera')return;
    const frame=camera.frame_number, deviceId=camera.device_id;
    previewPending=true;
    try{
      const response=await fetch('/api/camera/side-view/preview',{signal:AbortSignal.timeout(2000)});
      if(!response.ok)throw Error();
      const blob=await response.blob();
      if(!fresh||camera?.device_id!==deviceId)return;
      if(imageUrl)URL.revokeObjectURL(imageUrl);
      imageUrl=URL.createObjectURL(blob);image.src=imageUrl;
      image.hidden=false;$('side-placeholder').hidden=true;lastFrame=frame;
    }catch{clearPreview();}finally{previewPending=false;}
  }

  return {render(state){
    camera=state?.modules?.[key]?.state;
    fresh=!!(camera?.online&&camera.frame_age!==null&&camera.frame_age<1);
    locked=!!(camera?.recording||camera?.finalizing||['running','stopping'].includes(state?.experiment?.status));
    syncSettings();controls();
    const status=$('side-status');
    status.textContent=camera?.error?'故障':camera?.status||'服务离线';
    status.className='status '+(camera?.error?'error':fresh?'ready':'');
    $('side-nav-status').textContent='侧视 '+(camera?.error?'故障':fresh?'在线':'未接入');
    const measuring=fresh&&JSON.stringify(camera.settings)===JSON.stringify(camera.measurement_settings);
    const angle=camera?.measurement?.angle_deg;
    $('side-angle').textContent=measuring&&Number.isFinite(angle)?`${angle>0?'+':''}${angle.toFixed(1)}°`:'—';
    $('side-quality').textContent=measuring?(camera.measurement?.reason||'轮廓有效'):fresh?'应用设置中':'';
    const format=camera?.format;
    $('side-format').textContent=format?`${format.width} × ${format.height}${format.fps?` · ${format.fps.toFixed(1)} fps`:''}`:'—';
    $('side-age').textContent=Number.isFinite(camera?.frame_age)?`${camera.frame_age.toFixed(2)} s`:'—';
    const recording=camera?.recording?'记录中':camera?.finalizing?'收尾中':camera?.recorded_frames?'已记录':'未记录';
    $('side-record').textContent=recording+(camera?.recorded_frames?` · ${camera.recorded_frames} 帧`:'');
    if(fresh)preview();else clearPreview();
  }};
}
