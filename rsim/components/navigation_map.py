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


def laser_points(message, transform):
    """Use the TF at the scan's acquisition stamp, never a guessed extrinsic."""
    ranges = np.asarray(message['ranges'], float)
    angles = float(message['angle_min']) + np.arange(len(ranges)) * float(message['angle_increment'])
    valid = np.isfinite(ranges) & (ranges >= message['range_min']) & (ranges <= message['range_max'])
    if len(ranges) < 8 or not valid.any(): raise ValueError('laser has no usable returns')
    points = np.column_stack((ranges[valid] * np.cos(angles[valid]), ranges[valid] * np.sin(angles[valid]), np.zeros(valid.sum())))
    from scipy.spatial.transform import Rotation
    q = transform['quaternion']
    return (Rotation.from_quat(q).apply(points) + transform['translation'])[:, :2]
