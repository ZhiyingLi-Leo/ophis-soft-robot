"""Unmarked orange-root / white-tip segmentation. Depth curves are surface estimates."""
import math
from collections import deque


def thin(mask):
    import numpy as np
    image=np.pad((mask>0).astype(np.uint8),1)
    for _ in range(max(mask.shape)):
        changed=False
        for phase in (0,1):
            p=image[1:-1,1:-1]
            n=[image[:-2,1:-1],image[:-2,2:],image[1:-1,2:],image[2:,2:],
               image[2:,1:-1],image[2:,:-2],image[1:-1,:-2],image[:-2,:-2]]
            count=sum(n);transitions=sum(((n[i]==0)&(n[(i+1)%8]==1)).astype(np.uint8) for i in range(8))
            if phase==0:extra=(n[0]*n[2]*n[4]==0)&(n[2]*n[4]*n[6]==0)
            else:extra=(n[0]*n[2]*n[6]==0)&(n[0]*n[4]*n[6]==0)
            remove=(p==1)&(count>=2)&(count<=6)&(transitions==1)&extra
            if remove.any():p[remove]=0;changed=True
        if not changed:break
    return image[1:-1,1:-1]*255


def analyze(color, depth_mm, settings):
    import cv2
    import numpy as np
    h,w=color.shape[:2]
    roi=settings['roi']
    x0,y0,x1,y1=[int(v*n) for v,n in zip(roi,[w,h,w,h])]
    hsv=cv2.cvtColor(color,cv2.COLOR_BGR2HSV)
    orange=cv2.inRange(hsv,np.array([3,85,55]),np.array([27,255,255]))
    green=cv2.inRange(hsv,np.array([32,55,30]),np.array([95,255,255]))
    white=cv2.inRange(hsv,np.array([0,0,settings['white_min']]),np.array([179,settings['white_sat'],255]))
    allowed=np.zeros((h,w),np.uint8);allowed[y0:y1,x0:x1]=255
    orange &= allowed; white &= allowed
    kernel=np.ones((3,3),np.uint8)
    white=cv2.morphologyEx(white,cv2.MORPH_CLOSE,kernel)
    result=dict(valid=False, reason='未识别到橙色近端与白色柔性段', curve_px=[],
                projected_theta_deg=None, depth_fraction=0., surface_theta_deg=None,
                surface_curve_camera_mm=[], coordinate_frame='camera',
                measurement='未标定的可见表面估计；非三维中心线真值')
    overlay=color.copy()
    cv2.rectangle(overlay,(x0,y0),(x1-1,y1-1),(160,160,160),1)
    if np.count_nonzero(orange)<10: return result,overlay
    distances=cv2.distanceTransform(255-orange,cv2.DIST_L2,3)
    count,labels,stats,_=cv2.connectedComponentsWithStats(white)
    candidates=[]
    for label in range(1,count):
        if stats[label,cv2.CC_STAT_AREA]<30: continue
        region=labels==label
        distance=float(distances[region].min())
        if distance<=max(12,w*.04):candidates.append((distance,-stats[label,cv2.CC_STAT_AREA],label))
    if not candidates: return result,overlay
    label=min(candidates)[2]; mask=(labels==label).astype(np.uint8)*255
    skeleton=thin(mask)
    yy,xx=np.nonzero(skeleton);points=set(zip(xx.tolist(),yy.tolist()))
    if len(points)<8:return result,overlay
    root=min(points,key=lambda p:float(distances[p[1],p[0]]))
    queue=deque([root]);parent={root:None};distance={root:0}
    while queue:
        p=queue.popleft()
        for dx,dy in ((-1,-1),(0,-1),(1,-1),(-1,0),(1,0),(-1,1),(0,1),(1,1)):
            n=(p[0]+dx,p[1]+dy)
            if n in points and n not in parent:
                parent[n]=p;distance[n]=distance[p]+math.hypot(dx,dy);queue.append(n)
    tip=max(distance,key=distance.get);path=[]
    while tip is not None:path.append(tip);tip=parent[tip]
    path=np.array(path[::-1],dtype=float)
    if len(path)<10:return result,overlay
    def extend(endpoint,direction):
        norm=np.linalg.norm(direction)
        if norm<1e-9:return endpoint
        direction=direction/norm;last=endpoint.copy()
        for step in range(1,max(h,w)):
            q=endpoint+direction*step;x,y=np.rint(q).astype(int)
            if not (0<=x<w and 0<=y<h and mask[y,x]):break
            last=q
        return last
    path=np.vstack([extend(path[0],path[0]-path[min(4,len(path)-1)]),path,
                    extend(path[-1],path[-1]-path[max(0,len(path)-5)])])
    lengths=np.r_[0,np.cumsum(np.linalg.norm(np.diff(path,axis=0),axis=1))]
    query=np.linspace(0,lengths[-1],21)
    curve=np.column_stack([np.interp(query,lengths,path[:,j]) for j in (0,1)])
    def angle(a,b):
        norm=float(np.linalg.norm(a)*np.linalg.norm(b))
        return None if norm<1e-9 else float(np.degrees(np.arccos(np.clip(np.dot(a,b)/norm,-1,1))))
    result.update(valid=True,reason='二维轮廓已提取；深度需实物验证',curve_px=curve.tolist(),
                  projected_theta_deg=angle(curve[4]-curve[0],curve[-1]-curve[-5]))
    cv2.polylines(overlay,[curve.astype(np.int32)],False,(255,60,210),2)
    cv2.circle(overlay,tuple(curve[0].astype(int)),4,(255,100,0),-1)
    cv2.circle(overlay,tuple(curve[-1].astype(int)),4,(0,0,255),-1)
    if depth_mm is None:return result,overlay
    depths=[]
    for x,y in curve.astype(int):
        window=depth_mm[max(0,y-1):min(h,y+2),max(0,x-1):min(w,x+2)]
        selected=mask[max(0,y-1):min(h,y+2),max(0,x-1):min(w,x+2)]>0
        valid=window[(window>0)&selected]
        depths.append(float(np.median(valid)) if valid.size else None)
    result['depth_fraction']=sum(v is not None for v in depths)/len(depths)
    background=depth_mm[(green>0)&(allowed>0)&(depth_mm>0)]
    valid=[v for v in depths if v is not None]
    result['background_separation_mm']=float(np.median(background)-np.median(valid)) if background.size and valid else None
    result['sample_depth_mm']=depths
    return result,overlay
