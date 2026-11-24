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


def _components(mask, max_area, *, rod=False):
    import cv2
    import numpy as np

    _, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    parts = []
    for label, (x, y, width, height, area) in enumerate(stats[1:], 1):
        if not (10 if rod else 30) <= area <= max_area:
            continue
        rows, cols = np.where(labels[y:y+height, x:x+width] == label)
        pixels = np.column_stack((cols+x, rows+y))
        center, (across, along), angle = cv2.minAreaRect(pixels.astype(np.float32))
        if across > along:
            across, along = along, across
            angle -= 90
        length = along + 1
        thickness = area / length
        if length < (8 if rod else 10) or length < (1.55 if rod else 2.) * thickness:
            continue
        if rod and (across+1 > length/1.5 or across+1 > 2.4*thickness):
            continue
        # Cropping a desk or box can leave an apparently narrow strip along the ROI edge.
        if any(np.count_nonzero(coords == edge) > .65*length for coords, edge in
               ((pixels[:, 0], 0), (pixels[:, 0], mask.shape[1]-1),
                (pixels[:, 1], 0), (pixels[:, 1], mask.shape[0]-1))):
            continue
        theta = math.radians(angle)
        axis = np.array([-math.sin(theta), math.cos(theta)])
        ends = np.array(center) + np.array([[-1], [1]]) * along/2 * axis
        parts.append((pixels, thickness, ends))
    return parts


def _best_pair(rods, tips, saturation):
    import numpy as np

    best = None
    for shaft, rod_width, ends in rods:
        rod_strength = float(saturation[shaft[:, 1], shaft[:, 0]].mean()) / 255
        for tip, tip_width, _ in tips:
            width = max(rod_width, tip_width)
            ratio = width / min(rod_width, tip_width)
            if ratio > 2.8:
                continue
            for joint, back in (ends, ends[::-1]):
                offsets = tip-joint
                gap = np.linalg.norm(offsets, axis=1).min()
                if gap > max(4., 1.15*width):
                    continue
                axis = (joint-back)/np.linalg.norm(joint-back)
                forward = offsets @ axis
                start = forward.min()
                if np.median(forward) <= 0 or start < -width:
                    continue
                near_joint = offsets[forward <= start + 2*width]
                lateral = np.abs(near_joint @ np.array([-axis[1], axis[0]])).mean()
                score = (gap + lateral)/width + math.log(ratio) - rod_strength
                if best is None or score < best[0]:
                    best = (score, shaft, tip, joint, back, rod_width)
    return best


def _catheter(hsv, settings, frame_area):
    import cv2
    import numpy as np

    orange_min = settings.get('orange_min', [3, 85, 55])
    orange_max = tuple(settings.get('orange_max', [27, 255, 255]))
    white_min = settings['white_min']
    kernel = np.ones((3, 3), np.uint8)
    max_area = .2 * frame_area
    best = None
    best_rods = []
    # A brighter white mask separates the tube from dim reflections and the mat.
    # A more saturated orange mask can separate the shaft from cardboard at its edge.
    orange_levels = [orange_min[1]]
    if orange_min[1] < 200 <= orange_max[1]:
        orange_levels.append(200)
    previous_orange = None
    for saturation in orange_levels:
        orange = cv2.inRange(hsv, (orange_min[0], saturation, orange_min[2]), orange_max)
        if np.array_equal(orange, previous_orange):
            continue
        previous_orange = orange
        rods = _components(orange, max_area, rod=True)
        if not rods:
            continue
        previous_white = None
        for level in dict.fromkeys((white_min, max(white_min, 130))):
            white = cv2.inRange(hsv, (0, 0, level), (179, settings['white_sat'], 255))
            white[orange > 0] = 0
            if np.array_equal(white, previous_white):
                continue
            previous_white = white
            tips = _components(cv2.morphologyEx(white, cv2.MORPH_CLOSE, kernel), max_area)
            candidate = _best_pair(rods, tips, hsv[:, :, 1])
            if candidate is not None and (best is None or candidate[0] < best[0]):
                best, best_rods = candidate, rods
    if best is None:
        return None

    _, shaft, tip, joint, back, rod_width = best
    extent = [shaft, tip]
    axis = (back-joint)/np.linalg.norm(back-joint)
    cross = np.array([-axis[1], axis[0]])
    # Include the proximal shaft beyond a narrow mounting clip in the ROI.
    for fragment, width, ends in best_rods:
        if fragment is shaft or not .4*rod_width <= width <= 2.5*rod_width:
            continue
        delta = ends-back
        projected = delta @ axis
        if (-rod_width <= projected.min() <= 5*rod_width and projected.max() > 0
                and max(abs(delta @ cross)) <= max(3., rod_width, width)):
            extent.append(fragment)
    return shaft, tip, np.vstack(extent)


def _end_tangent(stations, pixels, width):
    import numpy as np

    direction = stations[-1] - stations[0]
    span = np.linalg.norm(direction)
    if span < 1e-9:
        return direction
    axis = direction / span
    normal = np.array([-axis[1], axis[0]])
    half_step = max(.75, .5*span/(len(stations)-1))
    # Refine the skeleton with transverse mask centroids before fitting each tangent.
    # This averages out one-pixel steps and excludes the rounded ends.
    centers = []
    for station in stations:
        offsets = pixels-station
        band = pixels[(abs(offsets @ axis) <= half_step) & (abs(offsets @ normal) <= 1.5*width)]
        centers.append(band.mean(axis=0) if len(band) else station)
    weights = np.arange(len(stations)) - (len(stations)-1)/2
    return weights @ np.asarray(centers)


def roi_pixel_bounds(roi, shape):
    height, width = shape[:2]
    x0, y0, x1, y1 = [int(v*n) for v, n in zip(roi, (width, height, width, height))]
    return x0, y0, max(x0+1, x1), max(y0+1, y1)


def analyze(color, depth_mm, settings, *, deproject=None, reconstruction=reconstruct, side_view=False):
    import cv2
    import numpy as np
    h,w=color.shape[:2]
    x0,y0,x1,y1=roi_pixel_bounds(settings['roi'], color.shape)
    hsv=cv2.cvtColor(color[y0:y1,x0:x1],cv2.COLOR_BGR2HSV)
    result=dict(valid=False, reason='未识别到橙色近端与白色柔性段', curve_px=[],
                projected_theta_deg=None, projected_signed_deg=None, depth_fraction=0., surface_theta_deg=None,
                surface_curve_camera_mm=[], coordinate_frame='camera',
                shape_3d={'valid':False,'angle_deg':None,'reason':'等待有效导管轮廓和对齐深度'},
                measurement='未标定的可见表面估计；非三维中心线真值')
    overlay=color.copy()
    cv2.rectangle(overlay,(x0,y0),(x1-1,y1-1),(160,160,160),1)
    selected = _catheter(hsv, settings, h*w)
    if selected is None:
        return result, overlay
    shaft, tip_pixels, extent = (points+[x0, y0] for points in selected)
    orange = np.zeros((h, w), np.uint8)
    orange[shaft[:, 1], shaft[:, 0]] = 255
    mask = np.zeros_like(orange)
    mask[tip_pixels[:, 1], tip_pixels[:, 0]] = 255
    x, y, width, height = cv2.boundingRect(tip_pixels)
    skeleton=thin(mask[y:y+height,x:x+width])
    yy,xx=np.nonzero(skeleton);points=set(zip((xx+x).tolist(),(yy+y).tolist()))
    if len(points)<8:return result,overlay
    distances = cv2.distanceTransform(255-orange, cv2.DIST_L2, 3)
    root=min(points,key=lambda p:(float(distances[p[1],p[0]]),p))
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
    def extend(endpoint,direction,*,supported=False):
        norm=np.linalg.norm(direction)
        if norm<1e-9:return endpoint
        direction=direction/norm;last=endpoint.copy()
        normal = np.array([-direction[1], direction[0]])
        for step in range(1,max(h,w)):
            q=endpoint+direction*step;x,y=np.rint(q).astype(int)
            if not (0<=x<w and 0<=y<h and mask[y,x]):break
            if supported and not all(0 <= px < w and 0 <= py < h and mask[py, px]
                                     for px, py in np.rint([q-normal, q+normal]).astype(int)):
                break
            last=q
        return last
    if not side_view:
        # Retrace the rounded cap along the local centerline, avoiding tiny skeleton hooks.
        tube_width = len(tip_pixels) / np.linalg.norm(np.diff(path,axis=0),axis=1).sum()
        cap_steps = min(len(path)-6, max(2, math.ceil(tube_width/2)))
        path = path[:-cap_steps]
        tip_direction = _end_tangent(path[-6:], tip_pixels, tube_width)
        tip = extend(path[-1], tip_direction, supported=True)
    else:
        tip = extend(path[-1],path[-1]-path[max(0,len(path)-5)])
    path=np.vstack([extend(path[0],path[0]-path[min(4,len(path)-1)]),path,
                    tip])
    lengths=np.r_[0,np.cumsum(np.linalg.norm(np.diff(path,axis=0),axis=1))]
    query=np.linspace(0,lengths[-1],21)
    curve=np.column_stack([np.interp(query,lengths,path[:,j]) for j in (0,1)])
    def angle(a,b):
        norm=float(np.linalg.norm(a)*np.linalg.norm(b))
        return None if norm<1e-9 else float(np.degrees(np.arccos(np.clip(np.dot(a,b)/norm,-1,1))))
    a, b = curve[4]-curve[0], curve[-1]-curve[-5]
    if side_view:
        tube_width = len(tip_pixels) / lengths[-1]
        a = _end_tangent(curve[1:8], tip_pixels, tube_width)
        b = _end_tangent(curve[-8:-1], tip_pixels, tube_width)
    projected=angle(a,b)
    signed=math.degrees(math.atan2(a[0]*b[1]-a[1]*b[0],np.dot(a,b))) if projected is not None else None
    if side_view and signed is not None and a[0] < 0:
        signed = -signed
    result.update(valid=True,reason='二维轮廓已提取；深度需实物验证',curve_px=curve.tolist(),
                  projected_theta_deg=projected, projected_signed_deg=signed)
    cv2.circle(overlay,tuple(curve[0].astype(int)),4,(255,100,0),-1)
    cv2.circle(overlay,tuple(curve[-1].astype(int)),4,(0,0,255),-1)
    if depth_mm is None:
        result['catheter_bounds_px'] = [*extent.min(axis=0).tolist(), *(extent.max(axis=0)+1).tolist()]
        return result,overlay
    backdrop = np.zeros((h,w),np.uint8)
    backdrop[y0:y1,x0:x1] = cv2.inRange(hsv,(32,55,30),(115,255,255))
    backdrop[(orange > 0) | (mask > 0)] = 0
    depths=[]
    for x,y in curve.astype(int):
        window=depth_mm[max(0,y-1):min(h,y+2),max(0,x-1):min(w,x+2)]
        selected=mask[max(0,y-1):min(h,y+2),max(0,x-1):min(w,x+2)]>0
        valid=window[(window>0)&selected]
        depths.append(float(np.median(valid)) if valid.size else None)
    result['depth_fraction']=sum(v is not None for v in depths)/len(depths)
    background=depth_mm[(backdrop>0)&(depth_mm>0)]
    valid=[v for v in depths if v is not None]
    result['background_separation_mm']=float(np.median(background)-np.median(valid)) if background.size and valid else None
    result['sample_depth_mm']=depths
    if deproject is not None:
        result['shape_3d']=reconstruction(depth_mm,orange,mask,backdrop,curve,deproject,settings)
    return result,overlay


def analyze_rgb(color, settings):
    import cv2
    import numpy as np

    height, width = color.shape[:2]
    working = color
    if max(height, width) > 640:
        scale = 640 / max(height, width)
        size = (max(1, round(width*scale)), max(1, round(height*scale)))
        working = cv2.resize(color, size, interpolation=cv2.INTER_AREA)
    measured, overlay = analyze(working, None, settings, side_view=True)
    angle = measured['projected_signed_deg']
    result = dict(valid=angle is not None, angle_deg=angle, curve_px=measured['curve_px'],
                  reason='' if angle is not None else measured['reason'])
    if 'catheter_bounds_px' in measured:
        result['catheter_bounds_px'] = measured['catheter_bounds_px']
    if working is not color:
        pixel_scale = np.array([width / working.shape[1], height / working.shape[0]])
        if result['curve_px']:
            # Resize maps pixel centers. Bounding boxes below use pixel edges.
            result['curve_px'] = ((np.asarray(result['curve_px'])+.5)*pixel_scale-.5).tolist()
        if 'catheter_bounds_px' in result:
            bounds = np.asarray(result['catheter_bounds_px']).reshape(2, 2)*pixel_scale
            result['catheter_bounds_px'] = [*np.floor(bounds[0]).astype(int).tolist(),
                                            *np.ceil(bounds[1]).astype(int).tolist()]
    return result, overlay


def locate_roi(color, settings, *, side_view=False):
    full_frame = dict(settings, roi=[0., 0., 1., 1.])
    if side_view:
        measured, _ = analyze_rgb(color, full_frame)
    else:
        measured, _ = analyze(color, None, full_frame)
    bounds = measured.get('catheter_bounds_px')
    if not measured['valid'] or not bounds:
        raise ValueError('未找到导管，请确认完整橙色近端与白色柔性段可见并调整颜色阈值')
    height, width = color.shape[:2]
    x0, y0, x1, y1 = bounds
    span = max(x1-x0, y1-y0)
    # Leave room for motion in both directions, including a nearly straight reference.
    if side_view:
        pad_x = max(.06, .4*span/width)
        pad_y = max(.15, .85*span/height)
    else:
        vertical = y1-y0 > x1-x0
        pad_x = max(.20, (.85 if vertical else .4)*span/width)
        pad_y = max(.25, (.4 if vertical else .85)*span/height)
    return [max(0., x0/width-pad_x), max(0., y0/height-pad_y),
            min(1., x1/width+pad_x), min(1., y1/height+pad_y)]


def locate_rgb_roi(color, settings):
    return locate_roi(color, settings, side_view=True)
