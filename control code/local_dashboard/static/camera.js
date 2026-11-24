import { createCameraShape } from './camera-shape.js';

const $=selector=>document.querySelector(selector);
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
  let state=null,busy=false,lastFrame=null,imageUrl=null;
  let setup=false,applied=false,drawing=null,previewSettings=null,confirming=false,experimentRunning=false,axesReady=false;
  const image=$('#cpreview'), box=$('#cropbox');
  const shapePanel=$('#cameraShapePanel');
  let shapeView=null, webglFailed=false;
  const roiIds=['cx0','cy0','cx1','cy1'];
  const device=(operation,arguments_={})=>act('device',{device:'camera.d405',operation,arguments:arguments_});
  $('#cconnect').onclick=()=>device(state?.online||state?.status==='连接中'||state?.status==='等待首帧'?'disconnect':'connect',{serial:$('#cserial').value.trim()});
  $('#crecord').onclick=()=>device('record_start',{label:$('#clabel').value.trim()});
  $('#crecordstop').onclick=()=>device('record_stop');
  async function apply(){
    applied=false; renderPretest();
    applied=await device('configure',{
      roi:roiIds.map(id=>Number($('#'+id).value)/100),
      white_min:Number($('#cwhite').value),white_sat:Number($('#csat').value)
    });
    renderPretest();
  }
  $('#capply').onclick=apply;
  for(const id of [...roiIds,'cwhite','csat'])$('#'+id).oninput=()=>{applied=false;renderPretest();};
  $('#pretestlimit').oninput=renderPretest;
  $('#pretestcancel').onclick=()=>{setup=false;renderPretest();showProgress();};
  $('#pretestconfirm').onclick=async()=>{
    if($('#pretestconfirm').disabled)return;
    confirming=true;renderPretest();
    const started=await act('pretest_start',{settings:state.settings,max_advance_mm:$('#pretestlimit').valueAsNumber});
    confirming=false;
    if(started){setup=false;showProgress();}
    renderPretest();
  };
  function renderPretest(){
    $('#pretestsetup').hidden=!setup;
    image.classList.toggle('drawing-enabled',setup&&!experimentRunning);
    if(!setup)return;
    const current=state?.online&&state.frame_age!==null&&state.frame_age<1;
    const settingsMatch=state&&JSON.stringify(state.settings)===JSON.stringify(previewSettings)
      &&JSON.stringify(state.settings)===JSON.stringify(state.measurement_settings);
    const limit=$('#pretestlimit').valueAsNumber;
    let hint='';
    if(!current)hint='接入 D405 并等待新鲜图像。';
    else if(!applied)hint='在图像上拖框，或输入边界后应用参数。';
    else if(!settingsMatch)hint='等待新识别区域的预览。';
    else if(!state.measurement?.shape_3d?.valid)hint=state.measurement?.shape_3d?.reason||'等待有效三维中心线。';
    else if(!axesReady)hint='请接入三根电缸，确认反馈有效且电缸静止。';
    else if(!(limit>=4&&limit<=48))hint='确认组合路径后，填写现场允许的总推进量上限。';
    $('#pretestready').textContent=hint||'三维测量有效，请检查中心线和夹角后确认。';
    $('#pretestconfirm').disabled=!!hint||confirming||experimentRunning||state.recording||!!drawing;
  }
  installRoiDrag(image, box, {
    enabled:()=>setup&&!experimentRunning&&!state?.recording,
    onDrag:active=>{drawing=active;renderPretest();},
    onSelection:bounds=>{
      if(!bounds){$('#pretestready').textContent='区域宽高至少占画面的 5%，请重新拖框。';return;}
      bounds.forEach((value,i)=>$('#'+roiIds[i]).value=(value*100).toFixed(2));
      apply();
    }
  });
  for(const b of document.querySelectorAll('[data-camera-command]'))b.onclick=()=>{
    $('#command').value=b.dataset.cameraCommand;$('#command').dispatchEvent(new Event('input'));$('#command').focus();
  };
  function clearPreview(){
    if(imageUrl)URL.revokeObjectURL(imageUrl);imageUrl=null;lastFrame=null;
    previewSettings=null;
    $('#cpreview').removeAttribute('src');$('#cpreview').hidden=true;$('#cplaceholder').hidden=false;
  }
  async function preview(c){
    if(busy||!c.online||c.frame_age===null||c.frame_age>2||document.body.dataset.page!=='camera'||lastFrame===c.frame_number)return;
    busy=true;
    try{
      const response=await fetch('/api/camera/preview',{signal:AbortSignal.timeout(2000)});
      if(!response.ok)throw Error('暂无新鲜图像');
      const blob=await response.blob();
      if(!state?.online)return;
      if(imageUrl)URL.revokeObjectURL(imageUrl);
      imageUrl=URL.createObjectURL(blob);
      const shown=JSON.parse(response.headers.get('X-Camera-Settings'));
      image.onload=()=>{previewSettings=shown;renderPretest();};
      image.src=imageUrl;
      $('#cpreview').hidden=false;$('#cplaceholder').hidden=true;lastFrame=c.frame_number;
    }catch{clearPreview();}finally{busy=false;}
  }
  const num=(v,d=1)=>Number.isFinite(v)?v.toFixed(d):'—';
  function renderShape(s, fresh){
    const cameraPage=document.body.dataset.page==='camera';
    const running=s?.experiment?.status==='running'&&s.experiment.pretest;
    const slot=cameraPage||!running?$('#cameraShapeSlot'):$('#pretestShapeSlot');
    if(shapePanel.parentElement!==slot)slot.append(shapePanel);
    const shape=fresh?s.camera.measurement?.shape_3d:null;
    if(!shapeView&&!webglFailed&&(cameraPage||running)){
      try {shapeView=createCameraShape($('#cameraShapeCanvas'));}
      catch {webglFailed=true;$('#cameraShapeCanvas').hidden=true;}
    }
    $('#cameraShapeStatus').textContent=webglFailed?'三维视图需要浏览器 WebGL 支持':shape?.valid?'':(shape?.reason||'等待有效三维中心线');
    if(cameraPage||running)shapeView?.update(shape,s?.camera.frame_number);
  }
  function render(s){
    const c=state=s?.camera;
    if(!c){clearPreview();$('#cstatus').textContent='服务离线';for(const id of ['cconnect','crecord','crecordstop','capply'])$('#'+id).disabled=true;renderPretest();renderShape(null,false);return;}
    const running=['running','stopping'].includes(s.experiment?.status);
    experimentRunning=running;
    axesReady=s.axes.every(a=>a.online&&!a.error&&!a.moving&&a.age!==null&&a.age<1);
    const fresh=c.online&&c.frame_age!==null&&c.frame_age<2;
    $('#cstatus').textContent=c.error?'故障':c.status;
    $('#cstatus').className='status '+(c.error?'error':fresh?'ready':'');
    $('#cnavstatus').textContent='D405 '+(fresh?'在线':c.error?'故障':'未接入');
    $('#cconnect').textContent=c.online||['连接中','等待首帧'].includes(c.status)?'断开 D405':'接入 D405';
    $('#cconnect').disabled=running||c.status==='正在断开';$('#capply').disabled=running||c.recording;
    $('#crecord').disabled=running||!fresh||c.recording;$('#crecordstop').disabled=running||!c.recording;
    const missing=Object.entries(c.dependencies).filter(([,v])=>!v).map(([k])=>k);$('#cdeps').textContent=missing.length?'缺少 '+missing.join('、'):'已就绪';
    $('#cerror').textContent=c.error||c.record_error||'';
    $('#cframes').textContent=String(c.processed_frames);$('#cage').textContent=num(c.frame_age,2)+' s';
    const shape=fresh?c.measurement?.shape_3d:null;
    $('#csurface').textContent=shape?.valid?num(shape.angle_deg)+'°':'—';
    $('#cdepth').textContent=fresh?`${shape?.observed_indices?.length??0} / 21`:'—';
    $('#crecordstate').textContent=c.recording?'记录中 · '+c.recorded_frames+' 帧':'已停止 · '+c.recorded_frames+' 帧';
    $('#csession').textContent=c.session_path||'记录将在本机 data/ 下创建实验目录';
    $('#summarycamera').textContent=fresh?'D405 · 采集中':c.error?'D405 · 接入故障':'D405 · 未接入';
    if(!fresh)clearPreview();else preview(c);
    renderPretest();
    renderShape(s,fresh);
  }
  return {render,beginPretest(){
    setup=true;applied=false;$('#pretestlimit').value='';
    if(state?.settings){
      state.settings.roi.forEach((v,i)=>$('#'+roiIds[i]).value=(v*100).toFixed(2));
      $('#cwhite').value=state.settings.white_min;$('#csat').value=state.settings.white_sat;
    }
    renderPretest();
  }};
}
