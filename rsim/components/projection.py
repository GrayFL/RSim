"""Timestamped poses and RGB projection of already-deskewed laser geometry."""
from bisect import bisect_left
from collections import deque
from dataclasses import dataclass

import numpy as np
from graphmap.pose import Pose
from scipy.spatial.transform import Rotation, Slerp


class PoseHistory:
    def __init__(self, *, capacity=1000, max_gap_s=.15):
        self.samples = deque(maxlen=capacity)
        self.max_gap_ns = int(max_gap_s * 1e9)

    def add(self, stamp_ns, pose):
        if self.samples and stamp_ns <= self.samples[-1][0]:
            return False
        self.samples.append((int(stamp_ns), pose))
        return True

    def at(self, stamp_ns):
        if not self.samples:
            return None
        stamps = [item[0] for item in self.samples]
        right = bisect_left(stamps, stamp_ns)
        if right < len(stamps) and stamps[right] == stamp_ns:
            return self.samples[right][1]
        if right == 0 or right == len(stamps):
            return None  # Never silently extrapolate an RGB pose.
        t0, p0 = self.samples[right - 1]
        t1, p1 = self.samples[right]
        if t1 - t0 > self.max_gap_ns:
            return None
        if (p0.wrd_frame, p0.ego_frame) != (p1.wrd_frame, p1.ego_frame):
            raise ValueError('pose history changed coordinate frames')
        alpha = (stamp_ns - t0) / (t1 - t0)
        rotation = Slerp([0., 1.], Rotation.from_quat([p0.quat, p1.quat]))(alpha)
        return Pose(position=p0.position * (1-alpha) + p1.position * alpha, rotation=rotation.as_quat(),
                    wrd_frame=p0.wrd_frame, ego_frame=p0.ego_frame)


@dataclass(frozen=True)
class PinholeCamera:
    width: int
    height: int
    matrix: np.ndarray
    distortion: np.ndarray
    model: str = 'plumb_bob'

    def __post_init__(self):
        matrix = np.asarray(self.matrix, dtype=np.float64).reshape(3, 3).copy()
        distortion = np.asarray(self.distortion, dtype=np.float64).reshape(-1).copy()
        if self.width <= 0 or self.height <= 0 or not np.isfinite(matrix).all() or not np.isfinite(distortion).all():
            raise ValueError('invalid camera calibration')
        if matrix[0, 0] <= 0 or matrix[1, 1] <= 0 or self.model not in ('', 'plumb_bob', 'rational_polynomial', 'equidistant'):
            raise ValueError('unsupported camera calibration or distortion model')
        object.__setattr__(self, 'matrix', matrix)
        object.__setattr__(self, 'distortion', distortion)


def colorize_laser(xyz_imu, *, scan_pose, image_pose, body_to_optical, camera, rgb,
                   occlusion_cell=2, occlusion_tolerance_m=.05, min_depth_m=.1):
    """Return laser geometry in RGB-time body frame, RGBA and source pixels.

    scan_pose: T_odom_imu at deskew end; image_pose: T_odom_base at RGB time.
    body_to_optical: T_base_optical (OpenCV +Z forward). No camera depth input.
    Occlusion is tested against the closest lidar return per small image cell;
    this cannot infer occluders that were never sampled by the laser.
    """
    xyz = np.asarray(xyz_imu)
    rgb = np.asarray(rgb)
    if xyz.ndim != 2 or xyz.shape[1] != 3 or not np.isfinite(xyz).all():
        raise ValueError('laser points must be finite Nx3')
    if rgb.shape != (camera.height, camera.width, 3) or rgb.dtype != np.uint8:
        raise ValueError('RGB pixels must match calibrated uint8 HxWx3 image')
    if occlusion_cell < 1 or occlusion_tolerance_m < 0 or min_depth_m <= 0:
        raise ValueError('invalid projection visibility parameters')
    T_body_imu = ~image_pose * scan_pose
    body_xyz = T_body_imu(xyz).astype(np.float32)
    optical_xyz = (~body_to_optical)(body_xyz)
    candidates = np.flatnonzero(optical_xyz[:, 2] > min_depth_m)
    rgba = np.zeros((len(xyz), 4), np.uint8)
    rgba[:, :3] = 128  # Alpha=0 explicitly marks unobserved color.
    pixels = np.full((len(xyz), 2), -1, np.int32)
    if not len(candidates):
        return body_xyz, rgba, pixels
    front = optical_xyz[candidates].astype(np.float64)
    if np.any(camera.distortion):
        import cv2
        if camera.model == 'equidistant':
            uv, _ = cv2.fisheye.projectPoints(front.reshape(1, -1, 3), np.zeros(3), np.zeros(3),
                camera.matrix, camera.distortion)
        else:
            uv, _ = cv2.projectPoints(front, np.zeros(3), np.zeros(3), camera.matrix, camera.distortion)
        uv = uv.reshape(-1, 2)
    else:
        homogeneous = front @ camera.matrix.T
        uv = homogeneous[:, :2] / homogeneous[:, 2:3]
    inside = np.isfinite(uv).all(axis=1) & (uv[:, 0] >= 0) & (uv[:, 0] < camera.width-.5)
    inside &= (uv[:, 1] >= 0) & (uv[:, 1] < camera.height-.5)
    candidates = candidates[inside]
    uv = np.floor(uv[inside] + .5).astype(np.int32)
    depth = front[inside, 2]
    cols = (camera.width + occlusion_cell - 1) // occlusion_cell
    rows = (camera.height + occlusion_cell - 1) // occlusion_cell
    cell = (uv[:, 1] // occlusion_cell) * cols + uv[:, 0] // occlusion_cell
    nearest = np.full(rows * cols, np.inf)
    np.minimum.at(nearest, cell, depth)
    visible = depth <= nearest[cell] + occlusion_tolerance_m
    candidates, uv = candidates[visible], uv[visible]
    rgba[candidates, :3] = rgb[uv[:, 1], uv[:, 0]]
    rgba[candidates, 3] = 255
    pixels[candidates] = uv[:, ::-1]
    return body_xyz, rgba, pixels
