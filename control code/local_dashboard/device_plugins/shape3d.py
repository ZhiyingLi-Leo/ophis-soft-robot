"""Reconstruct the flexible segment from aligned depth in camera millimetres."""

import math


def depth_at(depth, mask, green, pixel):
    import numpy as np

    x, y = np.rint(pixel).astype(int)
    if not (0 <= y < depth.shape[0] and 0 <= x < depth.shape[1]):
        return None
    window = np.s_[max(0,y-2):y+3, max(0,x-2):x+3]
    values = depth[window][(mask[window] > 0) & (depth[window] > 0)]
    if len(values) < 3:
        return None
    z = float(np.median(values))
    # The green backdrop lies behind the catheter in the measurement setup.
    nearby = np.s_[max(0,y-24):y+25, max(0,x-24):x+25]
    background = depth[nearby][(green[nearby] > 0) & (depth[nearby] > 0)]
    if len(background) >= 10 and z >= float(np.median(background)) - 1:
        return None
    return z


def orange_stations(orange, root, span_px):
    import cv2
    import numpy as np

    rows, columns = np.where(orange)
    if not len(rows):
        return []
    nearest = np.argmin((columns-root[0])**2 + (rows-root[1])**2)
    _, labels = cv2.connectedComponents(orange)
    component = labels[rows[nearest],columns[nearest]]
    same = labels[rows,columns] == component
    points = np.column_stack((columns[same], rows[same]))
    points = points[np.linalg.norm(points-root,axis=1) <= 1.25*span_px]
    if len(points) < 10:
        return []
    origin = points.mean(axis=0)
    _, _, basis = np.linalg.svd(points-origin,full_matrices=False)
    tangent = basis[0]
    if tangent @ (root-origin) < 0:
        tangent = -tangent
    s = (points-origin) @ tangent
    lo, hi = np.percentile(s,[8,92])
    if hi-lo < 8:
        return []
    return [np.median(band,axis=0) for position in np.linspace(lo,hi,9)
            if len(band := points[abs(s-position) <= max(1.5,(hi-lo)/18)])]


def fit_direction(points):
    import numpy as np

    origin = points.mean(axis=0)
    _, values, basis = np.linalg.svd(points-origin,full_matrices=False)
    if values[0] < 1 or values[1] > .35*values[0]:
        return None
    axis = basis[0]
    return axis if axis @ (points[-1]-points[0]) > 0 else -axis


def fit_shape(surface, shaft, diameter_mm, length_mm):
    import numpy as np

    observed = np.array([i for i, p in enumerate(surface) if p is not None])
    shape = dict(valid=False, angle_deg=None, centerline_camera_mm=[], observed_indices=observed.tolist(),
                 shaft_axis=None, tip_tangent=None, reason='三维重建不可用')
    if len(surface) < 7 or len(observed) < 7 or observed[0] != 0 or observed[-1] != len(surface)-1 or np.diff(observed).max() > 3:
        return dict(shape, reason='柔性段端点或连续区间缺少可靠深度')
    if len(shaft) < 5:
        return dict(shape, reason='橙色近端的可靠深度不足')

    shaft = np.asarray(shaft, dtype=float)
    observed_points = np.asarray([surface[i] for i in observed], dtype=float)
    if not np.isfinite(shaft).all() or not np.isfinite(observed_points).all():
        return dict(shape, reason='反投影返回无效三维点')
    direction = fit_direction(shaft)
    if direction is None:
        return dict(shape, reason='橙色近端的三维轴线无法可靠拟合')
    if np.max(np.linalg.norm(np.diff(observed_points,axis=0),axis=1)/np.diff(observed)) > diameter_mm:
        return dict(shape, reason='柔性段深度出现跳变')
    positions = np.arange(len(surface))
    path = np.column_stack([np.interp(positions, observed, values) for values in observed_points.T])
    path[1:-1] = (path[:-2]+2*path[1:-1]+path[2:])/4

    tangent = np.gradient(path,axis=0)
    tangent_norm = np.linalg.norm(tangent,axis=1)
    if np.any(tangent_norm < 1e-6):
        return dict(shape, reason='相邻三维点重合')
    tangent /= tangent_norm[:,None]
    # Estimate the section centre from its visible front surface and nominal radius.
    ray = path/np.linalg.norm(path,axis=1)[:,None]
    radial = ray - np.sum(ray*tangent,axis=1)[:,None]*tangent
    radial_norm = np.linalg.norm(radial,axis=1)
    if np.any(radial_norm < .2):
        return dict(shape, reason='局部切线接近视线，截面中心不可可靠恢复')
    center = path + (diameter_mm/2)*radial/radial_norm[:,None]
    arc = float(np.linalg.norm(np.diff(center,axis=0),axis=1).sum())
    if not .5*length_mm <= arc <= 1.7*length_mm:
        return dict(shape, reason='重建的柔性段长度与实物尺寸不符')

    # Fit the distal tangent from 3D neighbours and require actual distal observations.
    if np.count_nonzero((observed >= len(surface)-6) & (observed < len(surface)-1)) < 3:
        return dict(shape, reason='尖端切线附近实测点不足')
    tip = fit_direction(center[-6:-1])
    if tip is None:
        return dict(shape, reason='尖端的三维切线无法可靠拟合')
    angle = math.degrees(math.atan2(float(np.linalg.norm(np.cross(direction,tip))), float(direction @ tip)))
    return dict(shape, valid=True, reason='三维中心线估计有效', angle_deg=angle,
                centerline_camera_mm=center.tolist(), length_mm=arc,
                shaft_axis={'point_camera_mm':shaft[-1].tolist(), 'direction':direction.tolist()},
                tip_tangent=tip.tolist())


def reconstruct(depth, orange, white, green, curve, deproject, settings):
    def sample(pixels, mask):
        return [deproject(pixel.tolist(), z) if (z := depth_at(depth, mask, green, pixel)) is not None else None
                for pixel in pixels]

    import numpy as np
    shaft = orange_stations(orange, curve[0], np.linalg.norm(np.diff(curve,axis=0),axis=1).sum())
    return fit_shape(sample(curve, white), [p for p in sample(shaft, orange) if p is not None],
                     settings['diameter_mm'], settings['length_mm'])
