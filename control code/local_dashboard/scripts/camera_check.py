"""Read-only environment check. --probe enumerates cameras without starting streams."""
import argparse
import json
import platform
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from device_plugins.realsense import dependencies


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--probe',action='store_true',help='只读枚举 RealSense，不启动采集')
    args=parser.parse_args()
    deps=dependencies()
    report=dict(python=sys.executable,python_version=platform.python_version(),versions={},platform=platform.platform(),machine=platform.machine(),dependencies=deps,
                setup='使用工作台 .venv；numpy/opencv 见 requirements-camera.txt，macOS RealSense 见 CAMERA.md')
    for name,available in list(deps.items()):
        if available:
            try:
                module=__import__(name)
                report['versions'][name]=getattr(module,'__version__','unknown')
            except Exception as e:deps[name]=False;report[name+'_error']=str(e)
    if args.probe and deps['pyrealsense2']:
        import pyrealsense2 as rs
        try:
            report['devices']=[dict(name=d.get_info(rs.camera_info.name),serial=d.get_info(rs.camera_info.serial_number)) for d in rs.context().query_devices()]
            report['probe_status']='枚举成功' if report['devices'] else '枚举成功；当前未连接 RealSense 相机'
        except Exception as e:
            report['probe_error']=str(e)
    print(json.dumps(report,ensure_ascii=False,indent=2))
    return 3 if 'probe_error' in report else (0 if all(deps.values()) else 2)

if __name__=='__main__':sys.exit(main())
