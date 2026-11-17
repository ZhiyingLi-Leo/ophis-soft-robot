"""Export recorded measurement and timestamp quality to CSV, without hardware access."""
import argparse
import csv
import json
import math
from pathlib import Path


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session',type=Path,required=True)
    args=parser.parse_args()
    rows=[]
    with (args.session/'frames.jsonl').open() as f:
        for line in f:
            item=json.loads(line)
            if item['kind']!='frame':continue
            m=item['measurement'];stamp=item['host_receive_monotonic']
            row=dict(frame=item['frame_number'],host_monotonic=stamp,file=item['file'],
                     contour_valid=m['valid'],projected_theta_deg=m['projected_theta_deg'],
                     surface_theta_deg=m['surface_theta_deg'],depth_fraction=m['depth_fraction'],
                     background_separation_mm=m.get('background_separation_mm'))
            axes={a['id']:a for a in item.get('telemetry',{}).get('axes',[])}
            for i in (1,2,3):
                a=axes.get(i,{})
                row[f'q{i}_mm']=a.get('position')
                row[f'q{i}_target_mm']=a.get('target')
                row[f'q{i}_online']=a.get('online',False)
                row[f'q{i}_feedback_age_s']=stamp-a['last_seen'] if a.get('last_seen') else None
            rows.append(row)
    if not rows:raise ValueError('实验目录没有完整帧记录')
    output=args.session/'summary.csv'
    with output.open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=rows[0]);writer.writeheader();writer.writerows(rows)
    print(json.dumps(dict(csv=str(output),frames=len(rows),valid_contours=sum(r['contour_valid'] for r in rows),
                          note='surface_theta_deg 为相机坐标下未标定表面角；反馈年龄为有符号时间差，负值表示反馈晚于图像到达'),ensure_ascii=False))

if __name__=='__main__':main()
