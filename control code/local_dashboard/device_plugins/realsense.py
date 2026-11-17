"""Optional D405 plugin; SDK, capture and recording are isolated from actuator polling."""
import copy
import importlib.util
import json
import math
import shutil
import tempfile
import threading
import time
from datetime import datetime
from pathlib import Path

from .vision import analyze


def dependencies():
    result={}
    for name in ('numpy','cv2','pyrealsense2'):
        try: result[name]=importlib.util.find_spec(name) is not None
        except (ImportError,ValueError): result[name]=False
    return result


class D405Device:
    kind='camera'
    key='camera.d405'
    operations=('connect','disconnect','configure','record_start','record_stop','annotate')

    def __init__(self, data_root, telemetry):
        self.lock=threading.RLock()
        self.data_root=Path(data_root)
        self.telemetry=telemetry
        self.halt=threading.Event();self.thread=None
        self.jpeg=None;self.recording=None
        self.settings=dict(roi=[0.,0.,1.,1.],white_min=85,white_sat=100,length_mm=30.,diameter_mm=6.)
        self.status=dict(online=False,name='RealSense D405',status='未接入',error='',last_frame_time=None,
                         frame_number=None,frame_age=None,measurement=None,calibration=None,
                         processed_frames=0,recorded_frames=0,recording=False,session_path='',record_error='')

    def snapshot(self):
        with self.lock:
            s=copy.deepcopy(self.status)
            stamp=s['last_frame_time']
            s['frame_age']=None if stamp is None else max(0,time.monotonic()-stamp)
            s['settings']=copy.deepcopy(self.settings)
            s['dependencies']=dependencies()
            return s

    def preview(self):
        with self.lock:
            if not self.status['online'] or self.status['last_frame_time'] is None or time.monotonic()-self.status['last_frame_time']>2:return None
            return self.jpeg

    def command(self, operation, args, context=None):
        if operation not in self.operations:raise ValueError('相机不支持此操作')
        if operation=='connect':return self.connect(args.get('serial',''))
        if operation=='disconnect':return self.disconnect()
        if operation=='configure':
            with self.lock:
                if self.recording:raise ValueError('请先停止记录再修改识别参数')
                settings=copy.deepcopy(self.settings)
                if 'roi' in args:
                    roi=args['roi']
                    if not isinstance(roi,list) or len(roi)!=4 or not all(type(v) in (int,float) and math.isfinite(v) for v in roi):raise ValueError('ROI 需要四个有限数值')
                    if not 0<=roi[0]<roi[2]<=1 or not 0<=roi[1]<roi[3]<=1 or roi[2]-roi[0]<.05 or roi[3]-roi[1]<.05:raise ValueError('ROI 需在 0–1 内，宽高至少 0.05')
                    settings['roi']=roi
                for key in ('white_min','white_sat'):
                    if key in args:
                        v=args[key]
                        if type(v) is not int or not 0<=v<=255:raise ValueError('颜色阈值需要 0–255 整数')
                        settings[key]=v
                self.settings=settings
                return {'settings':copy.deepcopy(settings)}
        if operation=='record_start':return self.record_start(args.get('label',''))
        if operation=='record_stop':self.stop_recording('用户结束记录');return
        if operation=='annotate':
            with self.lock:
                if not self.recording:raise ValueError('尚未开始记录')
                note=args.get('note',{})
                if not isinstance(note,dict) or len(json.dumps(note))>2000:raise ValueError('样本注记需为短 JSON 对象')
                self.recording['index'].write(json.dumps(dict(kind='annotation',host_monotonic=time.monotonic(),note=note),ensure_ascii=False)+'\n')
                self.recording['index'].flush()

    def connect(self, serial=''):
        if not isinstance(serial,str) or len(serial)>100:raise ValueError('相机序列号无效')
        missing=[k for k,v in dependencies().items() if not v]
        if missing:raise ValueError('相机依赖未安装: '+', '.join(missing)+'；请运行 scripts/camera_check.py 检查环境')
        with self.lock:
            if self.thread and self.thread.is_alive():raise ValueError('相机正在连接或运行，请等待或先断开')
            self.halt=threading.Event();self.jpeg=None
            self.status.update(status='连接中',error='',measurement=None,last_frame_time=None)
            self.thread=threading.Thread(target=self._capture,args=(serial,),daemon=True)
            self.thread.start()

    def disconnect(self):
        self.halt.set()
        with self.lock:
            self.status.update(online=False,status='正在断开',measurement=None)
            self.jpeg=None
        self.stop_recording('相机断开')

    def close(self):
        self.disconnect()
        thread=self.thread
        if thread and thread is not threading.current_thread():thread.join(timeout=4)

    def record_start(self, label):
        if not isinstance(label,str) or len(label)>100:raise ValueError('实验名称最多 100 字')
        with self.lock:
            if not self.status['online'] or self.status['last_frame_time'] is None or time.monotonic()-self.status['last_frame_time']>1:
                raise ValueError('相机没有新鲜图像')
            if self.recording:raise ValueError('相机已经在记录')
            self.data_root.mkdir(parents=True,exist_ok=True)
            if shutil.disk_usage(self.data_root).free<512*1024**2:raise ValueError('可用空间不足 512 MB')
            path=Path(tempfile.mkdtemp(prefix=datetime.now().strftime('%Y%m%d_%H%M%S_'),dir=self.data_root))
            metadata=dict(label=label,created=datetime.now().astimezone().isoformat(),
                          settings=self.settings,calibration=self.status['calibration'],
                          depth_note='native_depth uses depth_scale_m; aligned_depth is unfilled and aligned to color',
                          shape_note='surface estimate in camera coordinates; no base registration or radius correction',
                          target_record_hz=5,timing_note='host receive time is not exposure time; per-axis feedback timestamps retained')
            (path/'metadata.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2))
            index=(path/'frames.jsonl').open('w')
            self.recording=dict(path=path,index=index,count=0,last_time=None)
            self.status.update(recording=True,session_path=str(path),recorded_frames=0,record_error='')
            return {'session_path':str(path)}

    def stop_recording(self, reason):
        with self.lock:
            if not self.recording:return
            rec=self.recording;self.recording=None;self.status['recording']=False
            try:
                rec['index'].write(json.dumps(dict(kind='end',reason=reason,frames=rec['count'],host_monotonic=time.monotonic()),ensure_ascii=False)+'\n')
            finally:rec['index'].close()

    def _save(self, arrays, info):
        import numpy as np
        # Never wait for the serial control lock while holding the camera lock.
        telemetry=self.telemetry()
        with self.lock:
            rec=self.recording
            if not rec or (rec['last_time'] is not None and info['host_receive_monotonic']-rec['last_time']<.2):return
            try:
                if shutil.disk_usage(rec['path']).free<512*1024**2:raise OSError('可用空间不足 512 MB')
                name=f"frame_{rec['count']:06d}.npz"
                np.savez_compressed(rec['path']/name,**arrays)
                rec['index'].write(json.dumps(dict(kind='frame',file=name,telemetry=telemetry,**info),ensure_ascii=False,allow_nan=False)+'\n')
                rec['index'].flush()
                rec['count']+=1;rec['last_time']=info['host_receive_monotonic']
                self.status['recorded_frames']=rec['count']
            except Exception as e:
                self.status['record_error']=str(e)
                self.stop_recording('记录失败: '+str(e))

    @staticmethod
    def _intrinsics(profile):
        i=profile.as_video_stream_profile().get_intrinsics()
        return dict(width=i.width,height=i.height,fx=i.fx,fy=i.fy,ppx=i.ppx,ppy=i.ppy,model=str(i.model),coeffs=list(i.coeffs))

    def _capture(self, serial):
        pipeline=None
        try:
            import numpy as np
            import cv2
            import pyrealsense2 as rs
            devices=[d for d in rs.context().query_devices() if 'D405' in d.get_info(rs.camera_info.name)]
            if serial:devices=[d for d in devices if d.get_info(rs.camera_info.serial_number)==serial]
            if len(devices)!=1:raise RuntimeError('未找到唯一 D405；检查连接，多个设备时填写序列号')
            device=devices[0];serial=device.get_info(rs.camera_info.serial_number)
            pipeline=rs.pipeline();config=rs.config();config.enable_device(serial)
            config.enable_stream(rs.stream.depth,640,480,rs.format.z16,30)
            config.enable_stream(rs.stream.color,640,480,rs.format.bgr8,30)
            profile=pipeline.start(config)
            scale=profile.get_device().first_depth_sensor().get_depth_scale()
            dp=profile.get_stream(rs.stream.depth);cp=profile.get_stream(rs.stream.color)
            extrinsic=dp.get_extrinsics_to(cp)
            calibration=dict(serial=serial,depth_scale_m=scale,depth=self._intrinsics(dp),color=self._intrinsics(cp),
                             depth_to_color=dict(rotation=list(extrinsic.rotation),translation_m=list(extrinsic.translation)),
                             firmware=device.get_info(rs.camera_info.firmware_version),stream_fps=cp.fps())
            with self.lock:self.status.update(calibration=calibration,status='等待首帧',processed_frames=0)
            align=rs.align(rs.stream.color);last_process=None
            while not self.halt.is_set():
                frames=pipeline.wait_for_frames(1500);received=time.monotonic()
                if last_process is not None and received-last_process<.15:continue
                last_process=received
                raw_depth=frames.get_depth_frame();raw_color=frames.get_color_frame()
                if not raw_depth or not raw_color:raise RuntimeError('图像流不完整')
                aligned=align.process(frames);depth=aligned.get_depth_frame()
                if not depth:raise RuntimeError('深度对齐失败')
                color=np.asanyarray(raw_color.get_data()).copy()
                native=np.asanyarray(raw_depth.get_data()).copy()
                aligned_array=np.asanyarray(depth.get_data()).copy()
                with self.lock:settings=copy.deepcopy(self.settings)
                measured,overlay=analyze(color,aligned_array.astype(float)*scale*1000,settings)
                intr=depth.profile.as_video_stream_profile().intrinsics
                points=[]
                for pixel,z in zip(measured['curve_px'],measured.get('sample_depth_mm',[])):
                    points.append(rs.rs2_deproject_pixel_to_point(intr,pixel,z) if z is not None else None)
                measured['surface_curve_camera_mm']=points
                if points and all(p is not None for p in points):
                    p=np.array(points);a=p[4]-p[0];b=p[-1]-p[-5];den=np.linalg.norm(a)*np.linalg.norm(b)
                    if den>1e-9:measured['surface_theta_deg']=float(np.degrees(np.arccos(np.clip(np.dot(a,b)/den,-1,1))))
                ok,jpeg=cv2.imencode('.jpg',overlay,[cv2.IMWRITE_JPEG_QUALITY,80])
                if not ok:raise RuntimeError('图像编码失败')
                info=dict(host_receive_monotonic=received,host_wall_time=time.time(),
                          frame_number=raw_depth.get_frame_number(),color_frame_number=raw_color.get_frame_number(),
                          depth_timestamp_ms=raw_depth.get_timestamp(),color_timestamp_ms=raw_color.get_timestamp(),
                          depth_clock=str(raw_depth.get_frame_timestamp_domain()),color_clock=str(raw_color.get_frame_timestamp_domain()),measurement=measured)
                with self.lock:
                    if self.halt.is_set():break
                    self.jpeg=jpeg.tobytes()
                    self.status.update(online=True,status='采集中',error='',last_frame_time=received,
                                       frame_number=info['frame_number'],measurement=measured)
                    self.status['processed_frames']+=1
                    recording=self.recording is not None
                if recording:self._save(dict(color_bgr=color,native_depth=native,aligned_depth=aligned_array),info)
        except Exception as e:
            with self.lock:self.status.update(online=False,error=str(e),status='连接失败 / 采集中断',measurement=None)
        finally:
            if pipeline is not None:
                try:pipeline.stop()
                except Exception:pass
            self.stop_recording('相机采集结束')
            with self.lock:
                self.status['online']=False;self.jpeg=None
                if self.halt.is_set():self.status['status']='未接入'
