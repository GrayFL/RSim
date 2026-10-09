import asyncio
import copy
import time
from types import SimpleNamespace

import numpy as np
import pytest
from graphmap.pose import Pose

from rsim.core import Component
from rsim.components.inertial import BodyIMU
from rsim.drivers.odometry import Odometry
from rsim.adapters.ros2.odometry import measurement_covariance


def imu_sample(attitude):
    return dict(header={'frame_id': 'imu'},
        orientation=dict(zip('xyzw', attitude.quat)), orientation_covariance=[0.]*9,
        angular_velocity=dict(x=0., y=0., z=0.), angular_velocity_covariance=[0.]*9,
        linear_acceleration=dict(zip('xyz', attitude.rot_mat.T @ [0., 0., 9.8])),
        linear_acceleration_covariance=[0.]*9)


def test_mount_and_full_attitude_remove_gravity_without_mutating_source():
    source = Component().signal('imu', clock='ros:system')
    mount = Pose(rotation=[15., -30., 90.], wrd_frame='base_footprint', ego_frame='imu')
    attitude = Pose(rotation=[20., -10., 45.], wrd_frame='imu_navigation', ego_frame='base_footprint')
    sensor_attitude = attitude * mount
    message = imu_sample(sensor_attitude)
    omega_body = np.array([.1, -.2, .3])
    bias = np.array([.01, .02, -.03])
    message['angular_velocity'] = dict(zip('xyz', mount.rot_mat.T @ omega_body + bias))
    original = copy.deepcopy(message)
    component = BodyIMU(source, mount, gyro_bias=bias)
    result = component.convert(message)
    assert result['header']['frame_id'] == 'base_footprint'
    np.testing.assert_allclose(list(result['orientation'].values()), attitude.quat, atol=1e-12)
    np.testing.assert_allclose(list(result['linear_acceleration'].values()), 0, atol=1e-12)
    np.testing.assert_allclose(list(result['angular_velocity'].values()), omega_body)
    np.testing.assert_allclose(np.diag(np.array(result['angular_velocity_covariance']).reshape(3,3)), .02**2, atol=1e-10)
    assert message == original


def test_unknown_orientation_does_not_become_zero_acceleration():
    component = BodyIMU(Component().signal('imu'), Pose(wrd_frame='body', ego_frame='imu'))
    value = imu_sample(Pose())
    value['orientation_covariance'][0] = -1.
    result = component.convert(value)
    assert result['linear_acceleration_covariance'][0] == -1
    value['orientation_covariance'][0] = 0.
    value['orientation'] = dict(x=0., y=0., z=0., w=0.)
    with pytest.raises(ValueError, match='quaternion'):
        component.convert(value)


def estimator(tmp_path, **kwargs):
    return Odometry(imu_mount=Pose(wrd_frame='base_footprint', ego_frame='imu'),
                    directory=tmp_path, **kwargs)


def test_native_filter_contract_avoids_duplicate_wheel_pose(tmp_path):
    import yaml
    c = estimator(tmp_path, parameters={'frequency': 40.})
    p = yaml.safe_load((tmp_path/'ekf-parameters.yaml').read_text())['/**']['ros__parameters']
    assert p['frequency'] == 40.
    assert [i for i, enabled in enumerate(p['odom0_config']) if enabled] == [6, 7, 11]
    assert [i for i, enabled in enumerate(p['imu0_config']) if enabled] == [5, 11, 12]
    assert p['smooth_lagged_data'] and p['imu0_relative']
    assert not p['publish_tf'] and not p['predict_to_current_time']
    assert c.scan is None
    with pytest.raises(ValueError, match='owns'):
        estimator(tmp_path, parameters={'publish_tf': True})
    with pytest.raises(ValueError, match='system-time'):
        estimator(tmp_path, wheel=Component().signal('wheel', clock='device:counter'))


def test_scan_estimator_is_independent_and_has_named_odometry(tmp_path):
    c = estimator(tmp_path, scan_topic='/test/scan', wheel_topic='/test/wheel',
        scan_mount=Pose(wrd_frame='base_footprint', ego_frame='laser'))
    assert c.scan.wheel_topic == '/test/wheel'
    assert c.scan.scan_topic == '/test/scan'
    assert c.scan.odom_frame == c.world_frame
    assert c.sources['scan'] is c.scan.odometry
    assert c.scan.driver.package == 'rtabmap_odom'


def test_covariance_rejects_negative_or_nonfinite_measurements():
    result = measurement_covariance([0.]*36, 6, [.1]*6)
    np.testing.assert_allclose(result, np.eye(6)*.01)
    invalid = np.eye(6); invalid[0, 1] = 2
    with pytest.raises(ValueError):
        measurement_covariance(invalid, 6, [.1]*6)


def test_native_filter_prediction_does_not_refresh_stale_sources(tmp_path):
    async def run():
        c = estimator(tmp_path)
        now = time.time_ns()
        frame = SimpleNamespace(sequence=1, stamp_ns=now)
        class Reader:
            output = SimpleNamespace(frames=[frame])
            async def get(self, **kwargs):
                return frame
        c.filtered = Reader()
        c.last_output_sequence = 0
        c.stamps = {'wheel': now-10**9, 'imu': now}
        await c.receive()
        assert c.last_output_stamp == -1
        assert not c.pose.frames
    asyncio.run(run())


def test_native_chassis_recipe_uses_no_internal_sharing(tmp_path):
    from rsim.drivers import NativeChassis
    robot = NativeChassis(stm32={'start_driver': False}, imu={'topic':'/test/imu'},
        imu_mount=Pose(wrd_frame='base_footprint', ego_frame='imu'), directory=tmp_path)
    assert robot.control.pose._target is robot.pose
    assert robot.control.velocity is robot.velocity
    from rsim.runtime.host import SharedSensor
    seen = set()
    def visit(c):
        if c in seen:
            return
        seen.add(c)
        assert not isinstance(c, SharedSensor)
        for child in c.dependencies:
            visit(child)
        for signal in c.inputs:
            visit(signal.producer)
    visit(robot.control)


def test_generic_chassis_service_accepts_pose_and_sink():
    from rsim.drivers import Chassis
    from rsim.components.simulated_chassis import SimulatedChassis
    from rsim.components.odometry import PlanarOdometry
    source = SimulatedChassis(noise=False, gyro_bias=0)
    estimate = PlanarOdometry(source.odom, source.imu)
    service = Chassis(pose=estimate.pose, velocity=source.velocity)
    assert service.controller.odometry is None
    assert service.controller.pose._target is estimate.pose
    assert service.controller.velocity is source.velocity


def test_config_loader_preserves_legacy_chassis_section(tmp_path, monkeypatch):
    from rsim.config import load_chassis
    import rsim.config.local_chassis as legacy
    path = tmp_path/'legacy.yaml'
    path.write_text('chassis: {port: device}\ncalibration: calibration.json\n')
    monkeypatch.setattr(legacy, 'load_local_chassis', lambda p, **kw: (p, kw))
    assert load_chassis(path) == (path, {'motion_enabled': False})


def test_native_mapping_inputs_keep_local_ros_signals(tmp_path):
    from rsim.adapters.ros2 import RosContext
    from rsim.adapters.ros2.mapping_input import MappingInputs
    inputs = MappingInputs(RosContext(), None, prefix='/test/map',
        lidar_driver=Component(), camera_driver=Component(), mounts=[],
        topics={'imu':'/imu','odom':'/wheel','scan':'/scan'},
        timing={'lidar':{'offset_s':0},'chassis':{'offset_s':0}},cloud_filter={})
    assert inputs.bridge is None
    assert {c.topic for c in inputs.remote.values()} == {'/imu','/wheel','/scan'}
    assert len(inputs.inputs) == 3
