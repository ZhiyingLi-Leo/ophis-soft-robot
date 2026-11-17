import * as THREE from 'three';
import { GLTFLoader } from './vendor/GLTFLoader.js';
import { OrbitControls } from './vendor/OrbitControls.js';
import { RoomEnvironment } from './vendor/RoomEnvironment.js';

const channelColors = [0x3989be, 0x2d9b91, 0xc89638];
// CAD lanes run bottom to top in the workstation view.
const cylinderIdByLane = [1, 2, 3];
export function createScene(onSelect) {
  const canvas = document.querySelector('#model');
  const stage = document.querySelector('#stage');
  const loading = document.querySelector('#loading');
  let renderer;
  try { renderer = new THREE.WebGLRenderer({canvas, antialias:true, alpha:false}); }
  catch (e) {
    loading.innerHTML = '<b>无法初始化 WebGL</b><small>请使用支持硬件加速的浏览器。控制面板仍可使用。</small>';
    return {update(){},view(){}};
  }
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = .88;
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0xf8fafc);
  const pmrem = new THREE.PMREMGenerator(renderer);
  const room = new RoomEnvironment();
  const environment = pmrem.fromScene(room, 0.04);
  scene.environment = environment.texture;
  room.dispose(); pmrem.dispose();
  scene.add(new THREE.HemisphereLight(0xffffff, 0xc4d0db, .8));
  const key = new THREE.DirectionalLight(0xfffaf2, 1.7);
  key.position.set(-.3,.8,.4); key.castShadow=true;
  key.shadow.mapSize.set(2048,2048);key.shadow.camera.left=-.6;key.shadow.camera.right=.6;
  key.shadow.camera.top=.45;key.shadow.camera.bottom=-.45;key.shadow.camera.near=.1;key.shadow.camera.far=2;
  key.shadow.bias=-.0001;key.shadow.normalBias=.0005;key.shadow.radius=4;scene.add(key);
  const fill = new THREE.DirectionalLight(0xdceaff, .8);fill.position.set(.4,.3,-.6);scene.add(fill);
  const ground = new THREE.Mesh(new THREE.PlaneGeometry(4,4), new THREE.ShadowMaterial({opacity:.12}));
  ground.rotation.x=-Math.PI/2;ground.position.y=-.001;ground.receiveShadow=true;scene.add(ground);
  const camera=new THREE.OrthographicCamera(-.5,.5,.3,-.3,.001,10);
  const controls=new OrbitControls(camera,canvas);controls.enableDamping=true;controls.dampingFactor=.09;
  controls.minZoom=.6;controls.maxZoom=6;controls.maxPolarAngle=Math.PI*.49;controls.minPolarAngle=.04;
  controls.target.set(-.01,.025,0);camera.position.set(.18,.5,.82);controls.update();
  const context = new THREE.Group(); scene.add(context);
  const roots={}, movers=[[],[],[]], selectable=[], anchors=[], leaders=[];
  let state=null,selected=0,mode='overview',loaded=false;
  const axisIndexForLane=lane=>state?state.axes.findIndex(a=>a.id===cylinderIdByLane[lane]):cylinderIdByLane[lane]-1;
  const laneForAxis=index=>cylinderIdByLane.indexOf(state?.axes[index]?.id??index+1);
  const material=(color,roughness=.45,metalness=.05)=>new THREE.MeshStandardMaterial({color,roughness,metalness});
  function tube(points,radius,mat){const path=new THREE.CatmullRomCurve3(points.map(p=>new THREE.Vector3(...p)));const mesh=new THREE.Mesh(new THREE.TubeGeometry(path,64,radius,12,false),mat);mesh.castShadow=true;mesh.receiveShadow=true;context.add(mesh);return mesh;}
  const hoseMaterials=channelColors.map(c=>material(c,.35,.03));
  // Schematic hydraulic paths connect the actual CAD assemblies, in metres.
  for(let i=0;i<3;i++) tube([[-.144,.030,(i-1)*.033],[-.123,.030,(i-1)*.033],[-.105,.052,(i-1)*.018],[-.09,.074,(i-1)*.004]],.0017,hoseMaterials[cylinderIdByLane[i]-1]);
  tube([[-.092,.074,0],[.02,.074,0],[.185,.074,0]],.006,material(0x242c35,.55));
  tube([[.185,.074,0],[.228,.074,0],[.267,.073,.006],[.303,.063,.027],[.322,.049,.052]],.0062,material(0x438fc1,.34));
  const tip=new THREE.Mesh(new THREE.SphereGeometry(.0058,20,12),material(0x2c3a43));tip.position.set(.322,.049,.052);context.add(tip);
  // Airway is context only; never derived from actuator state or presented as measured.
  const airwayMat=new THREE.MeshStandardMaterial({color:0xcbd7df,roughness:.85,transparent:true,opacity:.12,depthWrite:false,side:THREE.DoubleSide});
  tube([[.215,.074,0],[.27,.074,0],[.33,.093,-.045],[.37,.10,-.065]],.018,airwayMat).castShadow=false;
  tube([[.27,.074,0],[.31,.055,.043],[.35,.028,.088]],.018,airwayMat).castShadow=false;
  tube([[.325,.094,-.04],[.367,.10,.005]],.014,airwayMat).castShadow=false;
  function line(points,color=0xc8d2dc){const geo=new THREE.BufferGeometry().setFromPoints(points.map(p=>new THREE.Vector3(...p)));const l=new THREE.Line(geo,new THREE.LineBasicMaterial({color,transparent:true,opacity:.8}));scene.add(l);return l;}
  // Sparse engineering references, not a decorative grid.
  line([[-.37,.0005,.09],[.38,.0005,.09]],0xe0e6ec);
  for(const x of [-.36,-.26,-.16,-.06,.04,.14,.24,.34])line([[x,.0005,.087],[x,.0005,.093]],0xd2dce5);
  const ringmat=new THREE.MeshBasicMaterial({color:0x658aa9,transparent:true,opacity:.45,side:THREE.DoubleSide,depthWrite:false});
  const ring=new THREE.Mesh(new THREE.RingGeometry(.009,.010,48),ringmat);ring.rotation.x=-Math.PI/2;ring.visible=false;scene.add(ring);
  const indicatorMats=channelColors.map(c=>new THREE.MeshStandardMaterial({color:c,roughness:.55}));
  const lights=[];
  for(let i=0;i<3;i++){const dot=new THREE.Mesh(new THREE.SphereGeometry(.0028,16,12),material(0xaebac5));dot.position.set(-.335,.027,(i-1)*.033);scene.add(dot);lights.push(dot);}
  const pLight=new THREE.Mesh(new THREE.SphereGeometry(.003,16,12),material(0xaebac5));pLight.position.set(.06,.042,.053);scene.add(pLight);
  const namedAnchors=[['labelhydraulic',[-.245,.13,-.015],[-.245,.03,0]],['labeladvancer',[.055,.18,-.015],[.055,.075,0]],['labelcatheter',[.31,.16,-.005],[.3,.072,.025]]];
  for(const [id,p,to] of namedAnchors){anchors.push({el:document.querySelector('#'+id),point:new THREE.Vector3(...p)});leaders.push(line([to,[to[0],p[1]-.008,to[2]]],0xc3ced8));}
  const feedbackLabels=[],feedbackLines=[];
  const overlay=document.createElementNS('http://www.w3.org/2000/svg','svg');
  overlay.setAttribute('class','feedbacklines');stage.appendChild(overlay);
  for(let i=0;i<7;i++){
    const el=document.createElement('div');el.className='telemetrylabel';
    if(i<3)el.style.setProperty('--channel','#'+channelColors[cylinderIdByLane[2-i]-1].toString(16));
    stage.appendChild(el);feedbackLabels.push(el);const lead=document.createElementNS('http://www.w3.org/2000/svg','line');overlay.appendChild(lead);feedbackLines.push(lead);
  }
  function feedbackOverlay(){
    const width=stage.clientWidth,height=stage.clientHeight;
    feedbackLabels.forEach((el,i)=>{
      const axis=i<3;const lane=axis?2-i:null;const index=axis?axisIndexForLane(lane):i-3;
      el.hidden=!loaded||(axis?mode==='advancer':mode==='hydraulic');
      feedbackLines[i].style.display=el.hidden?'none':'';if(el.hidden)return;
      const point=axis?new THREE.Vector3(-.32,.04,(lane-1)*.033):new THREE.Vector3(index<2?-.035:.14,.10,index%2?-.022:.022);
      const v=point.project(camera);
      // Labels follow the workstation order: top ID 3, middle ID 2, bottom ID 1.
      const x=axis?Math.max(8,(v.x*.5+.5)*width-132):Math.min(width-145,(v.x*.5+.5)*width);
      const y=axis?height-122+i*36:height-48-(index%2)*43;
      el.style.left=Math.max(6,x)+'px';el.style.top=y+'px';
      const lead=feedbackLines[i];lead.setAttribute('x1',(v.x*.5+.5)*width);lead.setAttribute('y1',(-v.y*.5+.5)*height);lead.setAttribute('x2',Math.max(6,x)+20);lead.setAttribute('y2',y-18);
      if(axis){const a=state?.axes[index];const valid=a?.online&&!a.error&&a.age!==null&&a.age<2;
        el.innerHTML=`ID ${a?.id??cylinderIdByLane[lane]}<b>${valid?a.position.toFixed(3)+' mm':'— mm'}</b><small>${!state?'服务离线':a?.error?'故障':!a?.online?'未接入':!valid?'反馈过期':a.moving?'运动中 · 位置反馈':'位置反馈'}</small>`;
      }else{const p=state?.propulsion;const valid=p?.online&&!p.error&&p.telemetry_age!==null&&p.telemetry_age<2;const m=valid?p.motors?.[index]:null;
        el.innerHTML=`M${index+1}<b>${m?m.speed+' step/s':'—'}</b><small>${!state?'服务离线':p?.error?'板端故障':!p?.online?'未接入':p.backend==='nano'?'无逐电机遥测':m?m.current_ma+' mA · 电流设定':'等待反馈'}</small>`;
      }
    });
  }
  const loader=new GLTFLoader();
  function prepare(root){root.traverse(o=>{if(!o.isMesh)return;o.castShadow=true;o.receiveShadow=true;const m=o.material=mClone(o.material);m.envMapIntensity=.35;m.roughness=.55;m.metalness=.2;if(m.transparent){m.opacity=.32;m.depthWrite=false;m.roughness=.18;m.metalness=0;} });}
  function mClone(m){return m.clone();}
  function position(root,x){root.updateMatrixWorld(true);const box=new THREE.Box3().setFromObject(root);const c=box.getCenter(new THREE.Vector3());root.position.x+=x-c.x;root.position.y+=.004-box.min.y;root.position.z-=c.z;root.updateMatrixWorld(true);}
  async function load(){
    try{
      const [hyd,adv]=await Promise.all([loader.loadAsync('/models/cylinders.glb'),loader.loadAsync('/models/advancer.glb')]);
      roots.hydraulic=hyd.scene;roots.advancer=adv.scene;prepare(hyd.scene);prepare(adv.scene);
      hyd.scene.traverse(o=>{
        if(!o.isMesh)return;const center=new THREE.Box3().setFromObject(o).getCenter(new THREE.Vector3());
        const lane=Math.max(0,Math.min(2,Math.round((center.x+.021571)/.033)));
        o.userData.lane=lane;selectable.push(o);
        if(/slider_BD|REFERENCE_Plunger/.test(o.name)){movers[lane].push({mesh:o,z:o.position.z});}
        if(/REFERENCE_Barrel/.test(o.name)){o.material.color.setHex(0x64a8c5);o.material.opacity=.45;o.material.roughness=.28;}
        if(/REFERENCE_Plunger/.test(o.name)){o.material.color.setHex(0x9faebc);o.material.metalness=.3;}
        if(/slider_BD/.test(o.name)){o.material.color.setHex(channelColors[cylinderIdByLane[lane]-1]);o.material.roughness=.45;}
        if(/cylinder_BD/.test(o.name)){o.material.color.setHex(0x738493);o.material.metalness=.45;}
        if(/LAF16/.test(o.name)){o.material.color.setHex(0x343e48);o.material.metalness=.35;}
        if(/connector|holder/.test(o.name)){o.material.color.setHex(0xb9c3cd);o.material.metalness=.45;}
      });
      hyd.scene.rotation.y=-Math.PI/2;position(hyd.scene,-.245);scene.add(hyd.scene);
      // CAD coordinate system: Y = feed direction, X = normal to the base.
      adv.scene.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(new THREE.Vector3(0,1,0),new THREE.Vector3(1,0,0),new THREE.Vector3(0,0,-1)));
      position(adv.scene,.06);scene.add(adv.scene);
      adv.scene.traverse(o=>{if(!o.isMesh)return;if(/whole_base/.test(o.name)){o.material.color.setHex(0x9caebd);o.material.metalness=.45;}if(/cover/.test(o.name)){o.material.color.setHex(0xc6d0d9);o.material.roughness=.6;}selectable.push(o);o.userData.component='advancer';});
      loaded=true;loading.style.display='none';document.querySelector('#assetstatus').textContent='';
    }catch(e){loading.innerHTML='<b>装配模型加载失败</b><small>请确认本地 3d model 中的两份 GLB 文件存在。控制面板仍可使用。</small>';document.querySelector('#assetstatus').textContent='模型不可用';console.error(e);}
  }
  function view(name){mode=name;camera.zoom=1;let target,position,span;
    if(name==='hydraulic'){target=[-.245,.025,0];position=[-.10,.30,.36];span=.30;}
    else if(name==='advancer'){target=[.06,.05,0];position=[.23,.32,.36];span=.31;}
    else if(name==='top'){target=[-.005,.02,0];position=[-.005,.85,.0001];span=.84;}
    else{target=[-.005,.04,0];position=[.13,.45,.79];span=.86;}
    controls.target.set(...target);camera.position.set(...position);camera.userData.span=span;controls.update();resize();
    for(const b of document.querySelectorAll('[data-view]'))b.classList.toggle('active',b.dataset.view===name);
  }
  function resize(){const w=stage.clientWidth,h=stage.clientHeight;if(!w||!h)return;renderer.setSize(w,h,false);let span=camera.userData.span||.86;const aspect=w/h;const vertical=Math.max(span/aspect,.29*(mode==='overview'||mode==='top'?1:.7));camera.left=-vertical*aspect/2;camera.right=vertical*aspect/2;camera.top=vertical/2;camera.bottom=-vertical/2;camera.updateProjectionMatrix();}
  new ResizeObserver(resize).observe(stage);
  const raycaster=new THREE.Raycaster();let down=null;
  canvas.addEventListener('pointerdown',e=>down=[e.clientX,e.clientY]);
  canvas.addEventListener('pointerup',e=>{if(!down||Math.hypot(e.clientX-down[0],e.clientY-down[1])>5)return;const rect=canvas.getBoundingClientRect();raycaster.setFromCamera(new THREE.Vector2((e.clientX-rect.left)/rect.width*2-1,-(e.clientY-rect.top)/rect.height*2+1),camera);const hits=raycaster.intersectObjects(selectable.filter(o=>mode==='hydraulic'?o.userData.lane!==undefined:mode==='advancer'?o.userData.component==='advancer':true),false);if(hits.length){const o=hits[0].object;if(o.userData.lane!==undefined){selected=axisIndexForLane(o.userData.lane);onSelect(selected);}else if(o.userData.component==='advancer'){document.dispatchEvent(new CustomEvent('componentselect',{detail:'prop'}));}}});
  function render(){controls.update();
    if(loaded){roots.hydraulic.visible=mode!=='advancer';roots.advancer.visible=mode!=='hydraulic';context.visible=mode==='overview'||mode==='top';pLight.visible=mode!=='hydraulic';lights.forEach(l=>l.visible=mode!=='advancer');for(let i=0;i<3;i++){const a=state?.axes[axisIndexForLane(i)];const displacement=a?.online?Math.max(0,Math.min(a.stroke,a.position))/1000:0;for(const item of movers[i])item.mesh.position.z=item.z-displacement;lights[i].material.color.setHex(a?.error?0xb8514c:a?.moving?0xcb9639:a?.online?0x559985:0xbac4cc);}
      pLight.material.color.setHex(state?.propulsion.error?0xb8514c:state?.propulsion.speed?0xcb9639:state?.propulsion.online?0x559985:0xbac4cc);
      ring.visible=mode!=='advancer';ring.position.set(-.22,.0045,(laneForAxis(selected)-1)*.033);
    }
    anchors.forEach((a,i)=>{const v=a.point.clone().project(camera);const visible=loaded&&v.z>-1&&v.z<1&&(mode==='overview'||mode==='top');a.el.style.opacity=visible?'1':'0';leaders[i].visible=visible&&mode!=='top';a.el.style.left=(v.x*.5+.5)*stage.clientWidth+'px';a.el.style.top=(mode==='top'?60:(-v.y*.5+.5)*stage.clientHeight)+'px';});
    feedbackOverlay();renderer.render(scene,camera);requestAnimationFrame(render);
  }
  view('overview');load();render();
  return {view,update(s,index){state=s;selected=index;}};
}
