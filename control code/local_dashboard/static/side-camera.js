import { installRoiDrag, setupCameraPreview } from './camera.js';

const $=id=>document.getElementById(id);

export function setupSideCamera(act){
  const key='camera.side_view';
  const device=(operation,args={})=>act('device',{device:key,operation,arguments:args});
  const image=$('side-preview'), select=$('side-devices');
  let camera, fresh=false, locked=false, pending=false, listing=false;
  let settingsRoi;
  let error='';
  const preview=setupCameraPreview({
    image,card:$('side-card'),placeholder:$('side-placeholder'),endpoint:'/api/camera/side-view/preview',
    toggle:$('side-zoom'),reset:$('side-resetroi'),hint:$('side-zoomhint'),
    onReset:roi=>updateSettings('configure',{roi})
  });

  function activeConnection(){
    return camera?.online||['连接中','等待侧视首帧'].includes(camera?.status);
  }
  function controls(autoZoom=false){
    $('side-locate').disabled=!fresh||locked||pending;
    $('side-lock').hidden=!locked;
    $('side-discover').disabled=!camera||locked||listing;
    select.disabled=!camera||locked||listing||activeConnection();
    $('side-connect').disabled=!camera||locked||pending||(!activeConnection()&&!select.value);
    $('side-connect').textContent=activeConnection()?'断开侧视相机':'接入侧视相机';
    preview.update(camera&&settingsRoi,fresh,!!camera&&!locked&&!pending,autoZoom===true);
    $('side-error').textContent=error||camera?.error||camera?.record_error||'';
  }
  async function command(operation,args={}){
    pending=true;error='';controls();
    try{
      return await device(operation,args);
    }finally{pending=false;controls();}
  }
  async function updateSettings(operation,args,autoZoom=false){
    const result=await command(operation,args);
    if(result?.settings){
      settingsRoi=result.settings.roi;
      controls(autoZoom);
    }
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
  $('side-locate').onclick=()=>updateSettings('auto_locate',{},true);

  installRoiDrag(image,$('side-cropbox'),{
    enabled:preview.canDrag,
    onSelection:roi=>{
      if(!roi){error='区域宽高至少占画面的 5%。';controls();return;}
      return updateSettings('configure',{roi},true);
    }
  });

  return {render(state){
    camera=state?.modules?.[key]?.state;
    fresh=!!(camera?.online&&camera.frame_age!==null&&camera.frame_age<1);
    locked=!!(camera?.recording||camera?.finalizing||['running','stopping'].includes(state?.experiment?.status));
    settingsRoi=camera?.settings?.roi;
    controls();
    const status=$('side-status');
    status.textContent=camera?.error?'故障':camera?.status||'服务离线';
    status.className='status '+(camera?.error?'error':fresh?'ready':'');
    const measuring=fresh&&JSON.stringify(camera.settings)===JSON.stringify(camera.measurement_settings);
    const angle=camera?.measurement?.angle_deg;
    $('side-angle').textContent=measuring&&Number.isFinite(angle)?`${angle>0?'+':''}${angle.toFixed(1)}°`:'—';
    $('side-quality').textContent=measuring?(camera.measurement?.reason||'轮廓有效'):fresh?'应用设置中':'';
    const format=camera?.format;
    $('side-format').textContent=format?`${format.width} × ${format.height}${format.fps?` · ${format.fps.toFixed(1)} fps`:''}`:'—';
    $('side-age').textContent=Number.isFinite(camera?.frame_age)?`${camera.frame_age.toFixed(2)} s`:'—';
    const recording=camera?.recording?'记录中':camera?.finalizing?'收尾中':camera?.recorded_frames?'已记录':'未记录';
    $('side-record').textContent=recording+(camera?.recorded_frames?` · ${camera.recorded_frames} 帧`:'');
    preview.render(camera?.frame_number,camera?.device_id);
  }};
}
