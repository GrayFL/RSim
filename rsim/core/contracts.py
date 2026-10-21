"""SI and coordinate contracts for existing dictionary payloads.

These validators do not change the sealed wire codec. Source-dependent meaning
(gravity removal, truth/estimate and depth geometry) must be supplied explicitly;
it cannot be inferred from numbers. See docs/data-contracts.md.
"""
from typing import Literal, NotRequired, TypedDict
import numpy as np
from .model import Image
from .commands import VelocityCommand


class Vector3(TypedDict):
    x: float
    y: float
    z: float


class Quaternion(Vector3):
    w: float


class Header(TypedDict):
    frame_id: str
    stamp: NotRequired[dict]


class Imu(TypedDict):
    header: Header
    orientation: Quaternion
    orientation_covariance: list[float]
    angular_velocity: Vector3
    angular_velocity_covariance: list[float]
    linear_acceleration: Vector3
    linear_acceleration_covariance: list[float]


class Odometry(TypedDict):
    header: Header
    child_frame_id: str
    pose: dict
    twist: NotRequired[dict]


class LaserScan(TypedDict):
    header: Header
    angle_min: float
    angle_max: float
    angle_increment: float
    time_increment: float
    scan_time: float
    range_min: float
    range_max: float
    ranges: np.ndarray
    intensities: np.ndarray


def frame_id(value):
    if not isinstance(value, str) or not value:
        raise ValueError('frame_id must be a nonempty coordinate frame')


def vector(value, axes='xyz'):
    result = np.asarray([value[axis] for axis in axes], dtype=float)
    if not np.isfinite(result).all():
        raise ValueError('vector components must be finite')
    return result


def quaternion(value):
    result = vector(value, 'xyzw')
    if not np.isclose(np.linalg.norm(result), 1, atol=.01):
        raise ValueError('orientation requires a unit xyzw quaternion')
    return result


def covariance(value, size, *, unavailable=False):
    array = np.asarray(value, dtype=float)
    if array.shape != (size * size,) or not np.isfinite(array).all():
        raise ValueError(f'covariance must have {size * size} finite row-major entries')
    if unavailable and array[0] == -1:
        return False
    matrix = array.reshape(size, size)
    if not np.allclose(matrix, matrix.T, atol=1e-9) or np.linalg.eigvalsh(matrix).min() < -1e-9:
        raise ValueError('covariance must be symmetric positive semidefinite')
    # All zeros mean unknown, not a promise of zero uncertainty.
    return True


def validate_imu(data: Imu, *, gravity_removed: bool):
    if type(gravity_removed) is not bool:
        raise ValueError('gravity removal must be declared explicitly')
    frame_id(data['header']['frame_id'])
    for name in ('orientation', 'angular_velocity', 'linear_acceleration'):
        available = covariance(data[name + '_covariance'], 3, unavailable=True)
        if available:
            (quaternion if name == 'orientation' else vector)(data[name])
    return data


def validate_odometry(data: Odometry, *, pose_kind: Literal['estimate', 'ground_truth'],
                      velocity_frame: str | None = None):
    if pose_kind not in ('estimate', 'ground_truth'):
        raise ValueError('pose source must be estimate or ground_truth')
    frame_id(data['header']['frame_id'])
    frame_id(data['child_frame_id'])
    pose = data['pose']['pose']
    vector(pose['position'])
    quaternion(pose['orientation'])
    covariance(data['pose']['covariance'], 6)
    if 'twist' in data:
        if velocity_frame != data['child_frame_id']:
            raise ValueError('odometry twist must be expressed in the child frame')
        vector(data['twist']['twist']['linear'])
        vector(data['twist']['twist']['angular'])
        covariance(data['twist']['covariance'], 6)
    return data


def validate_scan(data: LaserScan):
    frame_id(data['header']['frame_id'])
    for name in ('angle_min', 'angle_max', 'angle_increment', 'time_increment', 'scan_time', 'range_min', 'range_max'):
        if not np.isfinite(data[name]):
            raise ValueError(f'{name} must be finite')
    if (data['time_increment'] < 0 or data['scan_time'] < 0 or data['range_min'] < 0
            or data['range_max'] <= data['range_min'] or data['angle_increment'] == 0):
        raise ValueError('invalid scan timing or range bounds')
    ranges, intensities = np.asarray(data['ranges']), np.asarray(data['intensities'])
    if ranges.ndim != 1 or intensities.ndim != 1 or len(intensities) not in (0, len(ranges)):
        raise ValueError('ranges and intensities must be matching vectors; intensities may be absent')
    if len(ranges) and not np.isclose(data['angle_min'] + (len(ranges)-1)*data['angle_increment'],
                                      data['angle_max'], atol=1e-5):
        raise ValueError('scan angles disagree with beam count')
    # NaN, infinities and out-of-range returns are retained; consumers mask them.
    return data


def validate_image(image: Image, *, camera_info=None, depth_unit=None, depth_type=None):
    frame_id(image.frame_id)
    formats = {'rgb8': ('u', 1, 3), 'bgr8': ('u', 1, 3), 'rgba8': ('u', 1, 4),
               'bgra8': ('u', 1, 4), 'mono8': ('u', 1, 1), '8UC1': ('u', 1, 1),
               'mono16': ('u', 2, 1), '16UC1': ('u', 2, 1), '32FC1': ('f', 4, 1)}
    if image.encoding not in formats:
        raise ValueError('unsupported image encoding')
    kind, size, channels = formats[image.encoding]
    pixels = image.pixels
    if (pixels.dtype.kind != kind or pixels.dtype.itemsize != size
            or pixels.ndim != (2 if channels == 1 else 3)
            or (channels > 1 and pixels.shape[-1] != channels)):
        raise ValueError('image shape/dtype disagrees with encoding')
    if depth_unit is not None or depth_type is not None:
        expected = {'16UC1': 'mm', '32FC1': 'm'}
        if expected.get(image.encoding) != depth_unit or depth_type not in ('optical_z', 'ray_distance'):
            raise ValueError('depth encoding, units and geometry must be explicit and compatible')
    if camera_info is not None:
        if (pixels.shape[:2] != (camera_info['height'], camera_info['width'])
                or camera_info['header']['frame_id'] != image.frame_id):
            raise ValueError('camera_info must match image size and optical frame')
        intrinsic = np.asarray(camera_info['K'], dtype=float)
        if intrinsic.size != 9 or not np.isfinite(intrinsic).all():
            raise ValueError('camera_info K must contain nine finite entries')
    return image


def validate_velocity(value):
    if not isinstance(value, VelocityCommand):
        raise TypeError('velocity requires VelocityCommand in body +X m/s and +Z rad/s')
    if not np.isfinite([value.linear_x, value.angular_z]).all():
        raise ValueError('velocity must be finite')
    return value
