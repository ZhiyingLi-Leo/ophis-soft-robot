import { createCameraShape } from './camera-shape.js';

const $=selector=>document.querySelector(selector);
export const fullFrameRoi=[0,0,1,1];
const sameRoi=(a,b)=>!!a&&!!b&&a.every((value,i)=>value===b[i]);

export function setupCameraPreview({image, placeholder, card, endpoint, toggle, reset, hint, onReset, onLoad=()=>{}}){
  let roi, fresh=false, configurable=false, zoom=false, fullFrameLoaded=false;
  let pending=false, lastFrame=null, imageUrl=null, deviceId;
  const canDrag=()=>fresh&&configurable&&!zoom&&fullFrameLoaded;
  function controls(){
    const hasRoi=!!roi&&!sameRoi(roi,fullFrameRoi);
    toggle.disabled=!hasRoi;
    toggle.textContent=zoom?'显示全景':'放大导管';
    reset.disabled=!hasRoi||!configurable;
    hint.hidden=!zoom;
    image.classList.toggle('drawing-enabled',canDrag());
  }
  function clear(){
    if(imageUrl)URL.revokeObjectURL(imageUrl);
    imageUrl=null;lastFrame=null;fullFrameLoaded=false;
    image.removeAttribute('src');image.hidden=true;placeholder.hidden=false;
    onLoad(null);
    controls();
  }
  toggle.onclick=()=>{
    zoom=!zoom;
    clear();
  };
  reset.onclick=()=>onReset(fullFrameRoi);
  return {
    canDrag,
    update(nextRoi, isFresh, canConfigure, autoZoom=false){
      const nextZoom=!!nextRoi&&!sameRoi(nextRoi,fullFrameRoi)&&(zoom||autoZoom);
      const changed=zoom!==nextZoom||(nextZoom&&!sameRoi(roi,nextRoi));
      zoom=nextZoom;roi=nextRoi;fresh=isFresh;configurable=canConfigure;
      if(changed||(!fresh&&imageUrl))clear();
      controls();
    },
    async render(frame, cameraId){
      if(deviceId!==cameraId){
        deviceId=cameraId;
        clear();
      }
      if(!fresh||pending||frame===lastFrame||card.hidden||document.body.dataset.page!=='camera')return;
      pending=true;
      const requestedZoom=zoom;
      try{
        const response=await fetch(endpoint+'?zoom='+(requestedZoom?1:0),{signal:AbortSignal.timeout(2000)});
        if(!response.ok)throw Error('暂无新鲜图像');
        const shown=JSON.parse(response.headers.get('X-Camera-Settings'));
        const blob=await response.blob();
        if(!fresh||deviceId!==cameraId||zoom!==requestedZoom||(requestedZoom&&!sameRoi(roi,shown.roi)))return;
        if(imageUrl)URL.revokeObjectURL(imageUrl);
        const url=imageUrl=URL.createObjectURL(blob);
        image.onload=()=>{
          if(imageUrl!==url)return;
          fullFrameLoaded=!requestedZoom;
          onLoad(shown);
          controls();
        };
        image.onerror=()=>{if(imageUrl===url)clear();};
        image.src=url;
        image.hidden=false;placeholder.hidden=true;lastFrame=frame;
      }catch{clear();}finally{pending=false;}
    }
  };
}

export function installRoiDrag(image, box, {enabled, onSelection, onDrag=()=>{}}){
  let drag=null;
  const clamp=value=>Math.max(0,Math.min(1,value));
  function coords(event, bounds){
    return [clamp((event.clientX-bounds.left)/bounds.width),clamp((event.clientY-bounds.top)/bounds.height)];
  }
  function boundsFor(event){
    const end=coords(event,drag.bounds), start=drag.start;
    return [Math.min(start[0],end[0]),Math.min(start[1],end[1]),Math.max(start[0],end[0]),Math.max(start[1],end[1])];
  }
  image.onpointerdown=event=>{
    if(!enabled()||drag||!image.naturalWidth||event.button!==0)return;
    const {left,top,width,height}=image.getBoundingClientRect();
    const scale=Math.min(width/image.naturalWidth,height/image.naturalHeight);
    if(!(scale>0))return;
    const w=image.naturalWidth*scale,h=image.naturalHeight*scale;
    const bounds={left:left+(width-w)/2,top:top+(height-h)/2,width:w,height:h};
    drag={id:event.pointerId,bounds,start:coords(event,bounds)};
    image.setPointerCapture(event.pointerId);
    onDrag(true);
  };
  image.onpointermove=event=>{
    if(drag?.id!==event.pointerId)return;
    const [x0,y0,x1,y1]=boundsFor(event), b=drag.bounds, container=image.parentElement;
    const parent=container.getBoundingClientRect();
    Object.assign(box.style,{left:`${b.left-parent.left-container.clientLeft+x0*b.width}px`,
      top:`${b.top-parent.top-container.clientTop+y0*b.height}px`,
      width:`${(x1-x0)*b.width}px`,height:`${(y1-y0)*b.height}px`});
    box.hidden=false;
  };
  function clear(event){
    if(drag?.id!==event.pointerId)return;
    drag=null;box.hidden=true;onDrag(false);
  }
  image.onpointercancel=clear;
  image.onlostpointercapture=clear;
  image.onpointerup=event=>{
    if(drag?.id!==event.pointerId)return;
    const bounds=boundsFor(event), [x0,y0,x1,y1]=bounds;
    clear(event);
    if(enabled())onSelection(x1-x0>=.05&&y1-y0>=.05?bounds:null);
  };
}

export function setupCamera(act, showProgress){
  let state=null,settingsRoi;
  let setup=false,applied=null,drawing=false,previewSettings=null,confirming=false,experimentRunning=false,axesReady=false;
  let listing=false,connecting=false,configuring=false,serialRequired=false,notice='',settingsSignature='';
  const image=$('#cpreview'), box=$('#cropbox'), select=$('#cdevices');
  const card=$('#d405-card'), shapeDrawer=$('#cameraShapeDrawer');
  const shapePanel=$('#cameraShapePanel');
  let shapeView=null, webglFailed=false;
  const roiIds=['cx0','cy0','cx1','cy1'];
  const device=(operation,arguments_={})=>act('device',{device:'camera.d405',operation,arguments:arguments_});
  const connected=()=>state?.online||['连接中','等待首帧'].includes(state?.status);
  const cameraFresh=()=>state?.online&&state.frame_age!==null&&state.frame_age<2;
  const canConfigure=()=>!!state&&!experimentRunning&&!state.recording&&!configuring&&!confirming;
  const preview=setupCameraPreview({
    image,card,placeholder:$('#cplaceholder'),endpoint:'/api/camera/preview',
    toggle:$('#czoom'),reset:$('#cresetroi'),hint:$('#czoomhint'),
    onReset:roi=>updateSettings('configure',{roi}),
    onLoad:settings=>{previewSettings=settings;renderPretest();}
  });
  function controls(autoZoom=false){
    const active=connected();
    $('#cdiscover').disabled=!state||experimentRunning||listing;
    select.disabled=!state||experimentRunning||listing||connecting||active;
    $('#cconnect').textContent=active?'断开 D405':'接入 D405';
    $('#cconnect').disabled=!state||experimentRunning||connecting||state.status==='正在断开'
      ||(!active&&(listing||(serialRequired&&!select.value)));
    $('#d405-config').disabled=!canConfigure();
    $('#clocate').disabled=!canConfigure()||!cameraFresh()||state.frame_age>=1||connecting;
    $('#crecord').disabled=experimentRunning||!cameraFresh()||state.recording;
    $('#crecordstop').disabled=experimentRunning||!state?.recording;
    preview.update(state&&settingsRoi,cameraFresh(),canConfigure(),autoZoom===true);
    $('#cerror').textContent=state?.error||state?.record_error||notice;
  }
  $('#cdiscover').onclick=async()=>{
    listing=true;notice='';controls();
    try{
      const response=await fetch('/api/camera/d405/devices',{signal:AbortSignal.timeout(5000)});
      const devices=await response.json();
      if(!response.ok)throw Error(devices.error);
      const selected=(connected()&&state?.calibration?.serial)||select.value;
      serialRequired=devices.length>1;
      select.replaceChildren(new Option(serialRequired?'请选择 D405':'自动匹配 · 单台 D405',''));
      for(const {serial,name,firmware} of devices){
        const option=new Option(`${name} · ${serial}`,serial);
        if(firmware)option.title=`固件 ${firmware}`;
        select.add(option);
      }
      if(devices.length===1)select.value=devices[0].serial;
      else if(devices.some(d=>d.serial===selected))select.value=selected;
      if(!devices.length)notice='SDK 未发现 D405。请检查 USB；单台相机可尝试自动匹配。';
    }catch(e){notice=e.message;}finally{listing=false;controls();}
  };
  select.onchange=controls;
  $('#cconnect').onclick=async()=>{
    connecting=true;notice='';controls();
    try{
      if(connected())await device('disconnect');
      else await device('connect',{serial:select.value});
    }finally{connecting=false;controls();}
  };
  $('#crecord').onclick=()=>device('record_start',{label:$('#clabel').value.trim()});
  $('#crecordstop').onclick=()=>device('record_stop');
  async function updateSettings(operation,args,autoZoom=false){
    configuring=true;applied=null;notice='';controls();renderPretest();
    let changed=false;
    try{
      const result=await device(operation,args);
      if(result?.settings){
        applied=result.settings;
        syncSettings(true,applied);
        changed=true;
      }
    }finally{configuring=false;controls(changed&&autoZoom);renderPretest();}
  }
  function apply(autoZoom=false){
    if(!canConfigure())return;
    const settings={
      roi:roiIds.map(id=>$('#'+id).valueAsNumber/100),
      white_min:$('#cwhite').valueAsNumber,white_sat:$('#csat').valueAsNumber
    };
    applied=null;
    if(![...settings.roi,settings.white_min,settings.white_sat].every(Number.isFinite)){
      notice='请填写全部 ROI 和颜色参数。';controls();renderPretest();return;
    }
    return updateSettings('configure',settings,autoZoom);
  }
  $('#clocate').onclick=()=>{if(!$('#clocate').disabled)return updateSettings('auto_locate',{},true);};
  $('#capply').onclick=()=>apply();
  for(const id of [...roiIds,'cwhite','csat'])$('#'+id).oninput=()=>{applied=null;renderPretest();};
  $('#pretestlimit').oninput=renderPretest;
  $('#pretestcancel').onclick=()=>{setup=false;renderPretest();showProgress();};
  $('#pretestconfirm').onclick=async()=>{
    if($('#pretestconfirm').disabled)return;
    confirming=true;controls();renderPretest();
    const started=await act('pretest_start',{settings:state.settings,max_advance_mm:$('#pretestlimit').valueAsNumber});
    confirming=false;
    if(started){setup=false;showProgress();}
    controls();renderPretest();
  };
  function renderPretest(){
    $('#pretestsetup').hidden=!setup;
    if(!setup)return;
    const current=cameraFresh()&&state.frame_age<1;
    const settingsMatch=applied&&state&&Object.entries(applied).every(([key,value])=>
      JSON.stringify(state.settings[key])===JSON.stringify(value))
      &&JSON.stringify(state.settings)===JSON.stringify(previewSettings)
      &&JSON.stringify(state.settings)===JSON.stringify(state.measurement_settings);
    const limit=$('#pretestlimit').valueAsNumber;
    let hint='';
    if(!current)hint='接入 D405 并等待新鲜图像。';
    else if(configuring)hint='正在应用识别参数。';
    else if(!applied)hint='自动定位、在图像上拖框，或输入边界后应用参数。';
    else if(!settingsMatch)hint='等待新识别区域的预览。';
    else if(!state.measurement?.shape_3d?.valid)hint=state.measurement?.shape_3d?.reason||'等待有效三维中心线。';
    else if(!axesReady)hint='请接入三根电缸，确认反馈有效且电缸静止。';
    else if(!(limit>=4&&limit<=48))hint='确认组合路径后，填写现场允许的总推进量上限。';
    $('#pretestready').textContent=hint||'三维测量有效，请检查中心线和夹角后确认。';
    $('#pretestconfirm').disabled=!!hint||confirming||experimentRunning||state.recording||!!drawing;
  }
  installRoiDrag(image, box, {
    enabled:preview.canDrag,
    onDrag:active=>{drawing=active;renderPretest();},
    onSelection:bounds=>{
      if(!bounds){notice='区域宽高至少占画面的 5%，请重新拖框。';controls();return;}
      bounds.forEach((value,i)=>$('#'+roiIds[i]).value=(value*100).toFixed(2));
      apply(true);
    }
  });
  const num=(v,d=1)=>Number.isFinite(v)?v.toFixed(d):'—';
  function renderShape(s, fresh){
    const page=document.body.dataset.page;
    const showProgress=page==='console'&&s?.experiment?.status==='running'&&s.experiment.pretest;
    const slot=showProgress?$('#pretestShapeSlot'):$('#cameraShapeSlot');
    if(shapePanel.parentElement!==slot)slot.append(shapePanel);
    if(!showProgress&&(page!=='camera'||card.hidden||!shapeDrawer.open))return;
    const shape=fresh?s.camera.measurement?.shape_3d:null;
    if(!shapeView&&!webglFailed){
      try {shapeView=createCameraShape($('#cameraShapeCanvas'));}
      catch {webglFailed=true;$('#cameraShapeCanvas').hidden=true;}
    }
    $('#cameraShapeStatus').textContent=webglFailed?'三维视图需要浏览器 WebGL 支持':shape?.valid?'':(shape?.reason||'等待有效三维中心线');
    shapeView?.update(shape,s?.camera.frame_number);
  }
  function syncSettings(force=false,settings=state?.settings){
    if(!settings)return;
    const signature=JSON.stringify(settings);
    if(!force&&signature===settingsSignature)return;
    settingsSignature=signature;
    settingsRoi=settings.roi;
    settings.roi.forEach((v,i)=>$('#'+roiIds[i]).value=+(v*100).toFixed(4));
    $('#cwhite').value=settings.white_min;$('#csat').value=settings.white_sat;
  }
  function render(s){
    const c=state=s?.camera;
    if(!c){
      controls();$('#cstatus').textContent='服务离线';$('#cstatus').className='status';
      renderPretest();renderShape(null,false);return;
    }
    const running=['running','stopping'].includes(s.experiment?.status);
    experimentRunning=running;
    axesReady=s.axes.every(a=>a.online&&!a.error&&!a.moving&&a.age!==null&&a.age<1);
    const fresh=cameraFresh();
    syncSettings();controls();
    $('#cstatus').textContent=c.error?'故障':c.status;
    $('#cstatus').className='status '+(c.error?'error':fresh?'ready':'');
    const missing=Object.entries(c.dependencies).filter(([,v])=>!v).map(([k])=>k);$('#cdeps').textContent=missing.length?'缺少 '+missing.join('、'):'已就绪';
    $('#cframes').textContent=String(c.processed_frames);$('#cage').textContent=Number.isFinite(c.frame_age)?num(c.frame_age,2)+' s':'—';
    const shape=fresh?c.measurement?.shape_3d:null;
    $('#csurface').textContent=shape?.valid?num(shape.angle_deg)+'°':'—';
    $('#cdepth').textContent=fresh?`${shape?.observed_indices?.length??0} / 21`:'—';
    $('#crecordstate').textContent=c.recording?'记录中 · '+c.recorded_frames+' 帧':c.recorded_frames?'已记录 · '+c.recorded_frames+' 帧':'未记录';
    $('#csession').textContent=c.session_path||'记录将在本机 data/ 下创建实验目录';
    preview.render(c.frame_number,c.calibration?.serial);
    renderPretest();
    renderShape(s,fresh);
  }
  return {render,beginPretest(){
    setup=true;applied=null;$('#pretestlimit').value='';shapeDrawer.open=true;
    syncSettings(true);
    renderPretest();
  }};
}
