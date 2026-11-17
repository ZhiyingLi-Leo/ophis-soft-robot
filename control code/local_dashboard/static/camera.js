const $=selector=>document.querySelector(selector);
export function setupCamera(act){
  let state=null,busy=false,lastFrame=null,imageUrl=null;
  const device=(operation,arguments_={})=>act('device',{device:'camera.d405',operation,arguments:arguments_});
  $('#cconnect').onclick=()=>device(state?.online||state?.status==='连接中'||state?.status==='等待首帧'?'disconnect':'connect',{serial:$('#cserial').value.trim()});
  $('#crecord').onclick=()=>device('record_start',{label:$('#clabel').value.trim()});
  $('#crecordstop').onclick=()=>device('record_stop');
  $('#capply').onclick=()=>device('configure',{
    roi:['cx0','cy0','cx1','cy1'].map(id=>Number($('#'+id).value)/100),
    white_min:Number($('#cwhite').value),white_sat:Number($('#csat').value)
  });
  for(const b of document.querySelectorAll('[data-camera-command]'))b.onclick=()=>{
    $('#command').value=b.dataset.cameraCommand;$('#command').dispatchEvent(new Event('input'));$('#command').focus();
  };
  function clearPreview(){
    if(imageUrl)URL.revokeObjectURL(imageUrl);imageUrl=null;lastFrame=null;
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
      imageUrl=URL.createObjectURL(blob);$('#cpreview').src=imageUrl;
      $('#cpreview').hidden=false;$('#cplaceholder').hidden=true;lastFrame=c.frame_number;
    }catch{clearPreview();}finally{busy=false;}
  }
  const num=(v,d=1)=>Number.isFinite(v)?v.toFixed(d):'—';
  function render(s){
    const c=state=s?.camera;
    if(!c){clearPreview();$('#cstatus').textContent='服务离线';for(const id of ['cconnect','crecord','crecordstop','capply'])$('#'+id).disabled=true;return;}
    const running=['running','stopping'].includes(s.experiment?.status);
    const fresh=c.online&&c.frame_age!==null&&c.frame_age<2;
    $('#cstatus').textContent=c.error?'故障':c.status;
    $('#cstatus').className='status '+(c.error?'error':fresh?'ready':'');
    $('#cnavstatus').textContent=fresh?'在线':c.error?'故障':'未接入';
    $('#cameradot').className='dot '+(c.error?'fault':fresh?'online':'');
    $('#cconnect').textContent=c.online||['连接中','等待首帧'].includes(c.status)?'断开 D405':'接入 D405';
    $('#cconnect').disabled=running||c.status==='正在断开';$('#capply').disabled=running||c.recording;
    $('#crecord').disabled=running||!fresh||c.recording;$('#crecordstop').disabled=running||!c.recording;
    const missing=Object.entries(c.dependencies).filter(([,v])=>!v).map(([k])=>k);$('#cdeps').textContent=missing.length?'缺少 '+missing.join('、'):'已就绪';
    $('#cerror').textContent=c.error||c.record_error||'';
    $('#cframes').textContent=String(c.processed_frames);$('#cage').textContent=num(c.frame_age,2)+' s';
    $('#cprojection').textContent=fresh?num(c.measurement?.projected_theta_deg)+'°':'—';
    $('#csurface').textContent=fresh?num(c.measurement?.surface_theta_deg)+'°':'—';
    $('#cdepth').textContent=fresh?num((c.measurement?.depth_fraction??0)*100)+'%':'—';
    $('#cquality').textContent=fresh?(c.measurement?.reason??'等待识别'):'等待新鲜图像';
    $('#crecordstate').textContent=c.recording?'记录中 · '+c.recorded_frames+' 帧':'已停止 · '+c.recorded_frames+' 帧';
    $('#csession').textContent=c.session_path||'记录将在本机 data/ 下创建实验目录';
    $('#summarycamera').textContent=fresh?'D405 · 采集中':c.error?'D405 · 接入故障':'D405 · 未接入';
    if(!fresh)clearPreview();else preview(c);
  }
  return {render};
}
