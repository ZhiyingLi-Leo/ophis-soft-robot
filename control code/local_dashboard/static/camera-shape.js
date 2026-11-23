import * as THREE from 'three';
import { OrbitControls } from './vendor/OrbitControls.js';

export function createCameraShape(canvas) {
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0xf6f8fb);
  const camera = new THREE.PerspectiveCamera(38, 1, .5, 500);
  camera.position.set(47, 35, 63);
  const renderer = new THREE.WebGLRenderer({canvas, antialias:true});
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  const controls = new OrbitControls(camera, canvas);
  controls.minDistance = 15;
  controls.maxDistance = 180;
  controls.update();

  const axes = new THREE.AxesHelper(12);
  const grid = new THREE.GridHelper(60, 12, 0xb9c9d8, 0xdce5ee);
  grid.position.y = -12;
  scene.add(axes, grid);

  const geometry = new THREE.Group();
  const centerline = new THREE.Mesh(new THREE.BufferGeometry(), new THREE.MeshBasicMaterial({color:0x16817c}));
  const reference = new THREE.ArrowHelper(new THREE.Vector3(1,0,0), new THREE.Vector3(), 12, 0xc77e35, 2.8, 1.4);
  const tangent = new THREE.ArrowHelper(new THREE.Vector3(1,0,0), new THREE.Vector3(), 10, 0xc84b55, 2.8, 1.4);
  const point = new THREE.SphereGeometry(.85, 10, 8);
  const root = new THREE.Mesh(point, new THREE.MeshBasicMaterial({color:0x386ca4}));
  const tip = new THREE.Mesh(point, new THREE.MeshBasicMaterial({color:0xc84b55}));
  geometry.add(centerline, reference, tangent, root, tip);
  geometry.visible = false;
  scene.add(geometry);

  function draw() {
    if (canvas.clientWidth && canvas.clientHeight) renderer.render(scene, camera);
  }
  new ResizeObserver(([entry]) => {
    const {width,height} = entry.contentRect;
    if (!width || !height) return;
    camera.aspect = width/height;
    camera.updateProjectionMatrix();
    renderer.setSize(width, height, false);
    draw();
  }).observe(canvas);
  controls.addEventListener('change', draw);

  const viewVector = ([x,y,z]) => new THREE.Vector3(x,-y,-z);
  let displayed = null;
  return {update(shape, frame) {
    if (!shape?.valid) {
      displayed = null;
      if (geometry.visible) {geometry.visible = false; draw();}
      return;
    }
    if (displayed === frame) return;
    displayed = frame;
    const origin = viewVector(shape.centerline_camera_mm[0]);
    const points = shape.centerline_camera_mm.map(p => viewVector(p).sub(origin));
    centerline.geometry.dispose();
    centerline.geometry = new THREE.TubeGeometry(new THREE.CatmullRomCurve3(points), 40, .48, 6, false);
    root.position.copy(points[0]);
    tip.position.copy(points.at(-1));
    const axis = viewVector(shape.shaft_axis.direction);
    reference.position.copy(viewVector(shape.shaft_axis.point_camera_mm).sub(origin).addScaledVector(axis,-12));
    reference.setDirection(axis);
    tangent.position.copy(tip.position);
    tangent.setDirection(viewVector(shape.tip_tangent));
    geometry.visible = true;
    draw();
  }};
}
