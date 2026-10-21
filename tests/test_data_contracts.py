"""ROS-free field/coordinate contracts; no conversion into new wire classes."""
import copy
import numpy as np
import pytest
from rsim.core.contracts import validate_imu, validate_scan, validate_image, validate_odometry
from rsim.core.model import Image


def imu():
    return dict(header={'frame_id': 'imu'}, orientation=dict(x=0., y=0., z=0., w=1.),
        orientation_covariance=[0.] * 9, angular_velocity=dict(x=0., y=0., z=.1),
        angular_velocity_covariance=[0.] * 9, linear_acceleration=dict(x=0., y=0., z=9.81),
        linear_acceleration_covariance=[0.] * 9)


def test_imu_requires_units_context_and_preserves_unavailable_fields():
    data = imu()
    assert validate_imu(data, gravity_removed=False) is data
    data['orientation'] = dict(x=0, y=0, z=0, w=0)
    with pytest.raises(ValueError, match='quaternion'):
        validate_imu(data, gravity_removed=False)
    data['orientation_covariance'][0] = -1
    validate_imu(data, gravity_removed=False)
    with pytest.raises(ValueError, match='gravity'):
        validate_imu(data, gravity_removed=None)


def test_scan_nonreturns_shape_and_timing():
    data = dict(header={'frame_id': 'laser'}, angle_min=-1., angle_max=1., angle_increment=.5,
                time_increment=.01, scan_time=.1, range_min=.05, range_max=10.,
                ranges=np.array([np.inf, 1., np.nan, 3., -np.inf]), intensities=np.array([]))
    assert validate_scan(data) is data
    with pytest.raises(ValueError, match='matching'):
        validate_scan(dict(data, intensities=np.ones(3)))
    with pytest.raises(ValueError, match='angles'):
        validate_scan(dict(data, angle_max=2.))


def test_image_depth_geometry_and_calibration_pairing():
    depth = Image(np.array([[0, 1, np.nan]], dtype=np.float32), '32FC1', 'camera_optical')
    info = dict(header={'frame_id': 'camera_optical'}, width=3, height=1, K=np.eye(3))
    assert validate_image(depth, camera_info=info, depth_unit='m', depth_type='optical_z') is depth
    with pytest.raises(ValueError, match='units'):
        validate_image(depth, depth_unit='mm', depth_type='optical_z')
    with pytest.raises(ValueError, match='match'):
        validate_image(depth, camera_info=dict(info, width=2))


def test_odometry_child_frame_and_covariance():
    data = dict(header={'frame_id': 'odom'}, child_frame_id='body',
        pose=dict(pose=dict(position=dict(x=0, y=0, z=0), orientation=dict(x=0, y=0, z=0, w=1)),
                  covariance=[0.] * 36))
    assert validate_odometry(data, pose_kind='estimate') is data
    data['twist'] = dict(twist=dict(linear=dict(x=0, y=0, z=0), angular=dict(x=0, y=0, z=0)), covariance=[0.]*36)
    with pytest.raises(ValueError, match='child frame'):
        validate_odometry(data, pose_kind='estimate', velocity_frame='odom')
    validate_odometry(data, pose_kind='estimate', velocity_frame='body')
