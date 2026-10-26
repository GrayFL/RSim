"""ROS-shaped occupancy and laser data to conservative metric obstacles."""
from __future__ import annotations
import hashlib
import io
import math
import numpy as np


def yaw_of(quaternion):
    if isinstance(quaternion, dict): quaternion = [quaternion[k] for k in ('x', 'y', 'z', 'w')]
    x, y, z, w = quaternion
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class OccupancyMap:
    def __init__(self, message, occupied_threshold=50):
        from scipy.ndimage import binary_dilation
        from pyrobo import Bounds2D
        info = message['info']
        self.resolution = float(info['resolution'])
        self.width, self.height = int(info['width']), int(info['height'])
        if not 0 < self.resolution < 10 or not 0 < self.width * self.height <= 16_000_000:
            raise ValueError('invalid or oversized map')
        self.grid = np.asarray(message['data'], dtype=np.int8).reshape(self.height, self.width).copy()
        self.grid.setflags(write=False)
        self.frame = message['header']['frame_id'].lstrip('/')
        origin = info['origin']
        self.origin = np.array([origin['position']['x'], origin['position']['y']], float)
        self.yaw = yaw_of(origin['orientation'])
        c, s = np.cos(self.yaw), np.sin(self.yaw)
        self.rotation = np.array([[c, -s], [s, c]])
        self.free = (self.grid >= 0) & (self.grid < occupied_threshold)
        # Keep every blocked cell bordering known free space. A cell is a disk
        # of radius resolution/sqrt(2), never a dimensionless surface sample.
        boundary = (~self.free) & binary_dilation(self.free, structure=np.ones((3, 3), bool))
        row, col = np.nonzero(boundary)
        self.obstacles = self.to_world(np.column_stack((col + .5, row + .5)))
        corners = self.to_world(np.array([[0, 0], [self.width, 0], [self.width, self.height], [0, self.height]]))
        self.bounds = Bounds2D(float(corners[:, 0].min()), float(corners[:, 0].max()),
                               float(corners[:, 1].min()), float(corners[:, 1].max()))
        self.point_radius = self.resolution / math.sqrt(2)
        digest = hashlib.sha256(self.grid.tobytes())
        digest.update(repr((info, self.frame)).encode())
        self.version = digest.hexdigest()[:20]
        self.png = None

    def to_world(self, cells):
        return np.asarray(cells) * self.resolution @ self.rotation.T + self.origin

    def to_grid(self, points):
        return np.floor((np.asarray(points) - self.origin) @ self.rotation / self.resolution).astype(int)

    def known_free(self, points):
        cells = self.to_grid(points)
        x, y = cells[..., 0], cells[..., 1]
        inside = (x >= 0) & (x < self.width) & (y >= 0) & (y < self.height)
        return inside & self.free[np.clip(y, 0, self.height - 1), np.clip(x, 0, self.width - 1)]

    def metadata(self):
        return dict(version=self.version, frame=self.frame, width=self.width, height=self.height,
                    resolution=self.resolution, origin=self.origin.tolist(), yaw=self.yaw)

    def image(self):
        if self.png is None:
            from PIL import Image
            colors = np.full(self.grid.shape, 110, np.uint8)
            colors[self.free] = 238
            colors[self.grid >= 50] = 25
            stream = io.BytesIO()
            Image.fromarray(np.flipud(colors)).save(stream, format='PNG', optimize=True)
            self.png = stream.getvalue()
        return self.png


def laser_points(message, transform, self_filter=None, *, allow_empty=False, body_transform=None, body_filter=None,
                 tray_filter=None):
    """Use the TF at the scan's acquisition stamp, never a guessed extrinsic."""
    ranges = np.asarray(message['ranges'], float)
    angles = float(message['angle_min']) + np.arange(len(ranges)) * float(message['angle_increment'])
    valid = np.isfinite(ranges) & (ranges >= message['range_min']) & (ranges <= message['range_max'])
    if self_filter:
        # Match only calibrated near-field self returns in sensor coordinates.
        # Preserve farther obstacles in the same direction, and all map cells.
        maximum = float(self_filter['max_range'])
        padding = int(self_filter.get('padding_beams', 0))
        if not math.isfinite(maximum) or not 0 < maximum <= .3 or not 0 <= padding <= 5:
            raise ValueError('invalid near-field self filter')
        angle_padding = padding * abs(float(message['angle_increment'])) + 1e-9
        for lower, upper in self_filter['angle_masks']:
            if not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper:
                raise ValueError('invalid self-filter angles')
            valid &= ~((ranges <= maximum) & (angles >= lower-angle_padding) & (angles <= upper+angle_padding))
        # Calibrated echoes at the edge of an existing hardware blind sector
        # need their own short range bound, not an enlarged whole-body mask.
        for edge in self_filter.get('edge_echo_masks', []):
            lower, upper, limit = [float(edge[k]) for k in ('lower', 'upper', 'max_range')]
            if not all(math.isfinite(v) for v in (lower, upper, limit)) or not 0 < upper-lower <= .04 or not 0 < limit <= .5:
                raise ValueError('invalid calibrated edge echo mask')
            valid &= ~((ranges <= limit) & (angles >= lower) & (angles <= upper))
    if len(ranges) < 8: raise ValueError('laser has no usable returns')
    if not valid.any():
        # A fresh auxiliary depth-to-scan frame can legitimately see no
        # obstacles: +inf marks no return. Do not confuse that with a missing
        # sensor, an all-invalid frame, or a required primary lidar failure.
        if allow_empty and np.isposinf(ranges).any() and not np.isneginf(ranges).any():
            return np.empty((0, 2))
        raise ValueError('laser has no usable returns')
    points = np.column_stack((ranges[valid] * np.cos(angles[valid]), ranges[valid] * np.sin(angles[valid]), np.zeros(valid.sum())))
    speckle_range = float((self_filter or {}).get('near_speckle_range', 0.))
    if not 0 <= speckle_range <= .6: raise ValueError('invalid near-field speckle range')
    if speckle_range:
        # Near-field multipath jumps around the taped shell; it is not confined
        # to a fixed strip. Require three neighboring beams within 25 mm.
        # Coherent surfaces (including the docks) and all farther returns stay.
        indices = np.flatnonzero(valid)
        lookup = np.full(len(ranges), -1, dtype=int)
        lookup[indices] = np.arange(len(points))
        neighbors = np.zeros(len(points), int)
        for shift in (-3, -2, -1, 1, 2, 3):
            other = lookup[(indices+shift) % len(ranges)]
            neighbors += (other >= 0) & (np.linalg.norm(points - points[np.maximum(other, 0)], axis=1) <= .025)
        points = points[(ranges[valid] > speckle_range) | (neighbors >= 2)]
    from scipy.spatial.transform import Rotation
    if body_filter:
        if body_transform is None: raise ValueError('missing sensor-to-body transform')
        body_points = Rotation.from_quat(body_transform['quaternion']).apply(points) + body_transform['translation']
        front, rear, width = [float(body_filter[k]) for k in ('front', 'rear', 'width')]
        padding = float(body_filter.get('padding', 0.))
        if not all(math.isfinite(v) and v > 0 for v in (front, rear, width)) or not 0 <= padding <= .02:
            raise ValueError('invalid calibrated body self filter')
        inside = ((body_points[:, 0] <= front+padding) & (body_points[:, 0] >= -rear-padding)
                  & (np.abs(body_points[:, 1]) <= width/2+padding))
        points = points[~inside]
    if tray_filter:
        # Only observed returns inside the physical rear tray are self returns.
        # Use the scan-time body transform, never a future predicted footprint;
        # a wall ahead of a moving tray must remain an obstacle.
        if body_transform is None: raise ValueError('missing sensor-to-body transform')
        inner, outer, width = [float(tray_filter[k]) for k in ('inner', 'outer', 'width')]
        if not all(math.isfinite(v) for v in (inner, outer, width)) or not 0 < inner < outer or width <= 0:
            raise ValueError('invalid physical tray filter')
        body_points = Rotation.from_quat(body_transform['quaternion']).apply(points) + body_transform['translation']
        inside = ((body_points[:, 0] <= -inner) & (body_points[:, 0] >= -outer)
                  & (np.abs(body_points[:, 1]) <= width / 2))
        points = points[~inside]
    q = transform['quaternion']
    return (Rotation.from_quat(q).apply(points) + transform['translation'])[:, :2]
