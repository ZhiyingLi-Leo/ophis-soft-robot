"""Unmarked orange-root / white-tip segmentation. Depth curves are surface estimates."""
import math
from collections import deque

from .shape3d import reconstruct


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


def analyze(color, depth_mm, settings, *, deproject=None, reconstruction=reconstruct):
    import cv2
    import numpy as np
    h,w=color.shape[:2]
    roi=settings['roi']
    x0,y0,x1,y1=[int(v*n) for v,n in zip(roi,[w,h,w,h])]
    x1,y1=max(x0+1,x1),max(y0+1,y1)
    hsv=cv2.cvtColor(color[y0:y1,x0:x1],cv2.COLOR_BGR2HSV)
    orange=np.zeros((h,w),np.uint8)
    orange[y0:y1,x0:x1]=cv2.inRange(hsv,np.array(settings.get('orange_min',[3,85,55])),
                                      np.array(settings.get('orange_max',[27,255,255])))
    white=np.zeros((h,w),np.uint8)
    white[y0:y1,x0:x1]=cv2.inRange(hsv,np.array([0,0,settings['white_min']]),
                                     np.array([179,settings['white_sat'],255]))
    if 'orange_min' in settings:
        white[orange>0]=0
    allowed=np.zeros((h,w),np.uint8);allowed[y0:y1,x0:x1]=255
    orange &= allowed; white &= allowed
    kernel=np.ones((3,3),np.uint8)
    white=cv2.morphologyEx(white,cv2.MORPH_CLOSE,kernel)
    if 'orange_min' in settings:
        white &= allowed
    result=dict(valid=False, reason='未识别到橙色近端与白色柔性段', curve_px=[],
                projected_theta_deg=None, projected_signed_deg=None, depth_fraction=0., surface_theta_deg=None,
                surface_curve_camera_mm=[], coordinate_frame='camera',
                shape_3d={'valid':False,'angle_deg':None,'reason':'等待有效导管轮廓和对齐深度'},
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
    x,y,width,height=stats[label,:4]
    skeleton=thin(mask[y:y+height,x:x+width])
    yy,xx=np.nonzero(skeleton);points=set(zip((xx+x).tolist(),(yy+y).tolist()))
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
    a,b=curve[4]-curve[0],curve[-1]-curve[-5]
    projected=angle(a,b)
    signed=math.degrees(math.atan2(a[0]*b[1]-a[1]*b[0],np.dot(a,b))) if projected is not None else None
    result.update(valid=True,reason='二维轮廓已提取；深度需实物验证',curve_px=curve.tolist(),
                  projected_theta_deg=projected, projected_signed_deg=signed)
    cv2.circle(overlay,tuple(curve[0].astype(int)),4,(255,100,0),-1)
    cv2.circle(overlay,tuple(curve[-1].astype(int)),4,(0,0,255),-1)
    if depth_mm is None:
        if 'orange_min' in settings:
            _,_,orange_stats,_=cv2.connectedComponentsWithStats(orange)
            root=np.asarray(curve[0])
            local=[]
            for index, (ox,oy,ow,oh,area) in enumerate(orange_stats[1:],1):
                if area<10:continue
                nearest=np.clip(root,[ox,oy],[ox+ow-1,oy+oh-1])
                local.append((float(np.linalg.norm(root-nearest)),-area,index))
            if local:
                ox,oy,ow,oh=orange_stats[min(local)[2],:4]
                result['catheter_bounds_px']=[int(v) for v in
                    (min(x,ox),min(y,oy),max(x+width,ox+ow),max(y+height,oy+oh))]
        return result,overlay
    green=np.zeros((h,w),np.uint8)
    green[y0:y1,x0:x1]=cv2.inRange(hsv,np.array([32,55,30]),np.array([95,255,255]))
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
    if deproject is not None:
        result['shape_3d']=reconstruction(depth_mm,orange,mask,green & allowed,curve,deproject,settings)
    return result,overlay


def analyze_rgb(color, settings):
    measured, overlay = analyze(color, None, settings)
    angle = measured['projected_signed_deg']
    result = dict(valid=angle is not None, angle_deg=angle, curve_px=measured['curve_px'],
                  reason='' if angle is not None else measured['reason'])
    if 'catheter_bounds_px' in measured:
        result['catheter_bounds_px'] = measured['catheter_bounds_px']
    return result, overlay


def locate_rgb_roi(color, settings):
    measured, _ = analyze_rgb(color, dict(settings, roi=[0., 0., 1., 1.]))
    bounds = measured.get('catheter_bounds_px')
    if not measured['valid'] or not bounds:
        raise ValueError('未找到导管，请确认完整橙色近端与白色柔性段可见并调整颜色阈值')
    height, width = color.shape[:2]
    x0, y0, x1, y1 = bounds
    span = max(x1-x0, y1-y0)
    # Leave room for motion in both directions, including a nearly straight reference.
    pad_x = max(.06, .4*span/width)
    pad_y = max(.15, .85*span/height)
    return [max(0., x0/width-pad_x), max(0., y0/height-pad_y),
            min(1., x1/width+pad_x), min(1., y1/height+pad_y)]
