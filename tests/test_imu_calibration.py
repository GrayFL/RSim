import asyncio
import copy

import numpy as np
import pytest
from graphmap.pose import Pose

from rsim import Component, Runtime
from rsim.components.imu_calibration import PlanarIMU, PlanarIMUCalibration, calibrate_planar_imu
from rsim.components.simulated_chassis import imu_message


def windows():
    rng = np.random.default_rng(4)
    mount = Pose(roll=70, pitch=-40, yaw=80)
    bias = np.array([.004, -.002, .007])
    local, reference = [], []
    for _ in range(300):
        msg = imu_message(0., frame='external')
        msg['linear_acceleration'] = dict(zip('xyz', mount.rot_mat.T @ [0, 0, 9.81] + rng.normal(0,.01,3)))
        msg['linear_acceleration_covariance'] = [0.] * 9
        msg['angular_velocity'] = dict(zip('xyz', bias + rng.normal(0,.0004,3)))
        local.append(msg)
        ref = imu_message(0., frame='reference')
        ref['linear_acceleration'] = dict(zip('xyz', [0., 0., 9.8]))
        ref['linear_acceleration_covariance'] = [0.] * 9
        reference.append(ref)
    return local, reference, mount, bias


def test_projection_corrects_arbitrary_tilt_and_bias_without_inventing_yaw():
    local, reference, mount, bias = windows()
    cal = calibrate_planar_imu(local, reference,
                              reference_to_body=Pose(wrd_frame='body', ego_frame='reference'))
    assert np.linalg.norm(np.array(cal.gyro_bias) - bias) < .0001
    np.testing.assert_allclose(cal.tilt_pose().rot_mat @ cal.vertical_axis, [0,0,1], atol=1e-12)
    assert PlanarIMUCalibration.from_dict(cal.to_dict()) == cal

    async def run():
        source = Component()
        raw = source.signal('raw', clock='test')
        planar = PlanarIMU(raw, cal)
        async with Runtime(planar):
            message = copy.deepcopy(local[0])
            message['angular_velocity'] = dict(zip('xyz', mount.rot_mat.T @ [0,0,-.25] + bias))
            await raw.publish(message, stamp_ns=123, clock='test', received_ns=456)
            frame = await planar.get(timeout=1)
            assert (frame.stamp_ns, frame.received_ns, frame.clock) == (123,456,'test')
            assert frame.data['header']['frame_id'] == 'body'
            assert abs(frame.data['angular_velocity']['z'] + .25) < .0001
            assert frame.data['orientation_covariance'][0] == -1
            assert not frame.data['metadata']['yaw_extrinsic_known']
    asyncio.run(run())


def test_reject_motion_nonlevel_and_invalid_calibration():
    local, reference, _, _ = windows()
    mount = Pose(wrd_frame='body', ego_frame='reference')
    with pytest.raises(ValueError, match='100'):
        calibrate_planar_imu(local[:10], reference, reference_to_body=mount)
    local[50]['angular_velocity']['z'] = .1
    with pytest.raises(ValueError, match='stationary'):
        calibrate_planar_imu(local, reference, reference_to_body=mount)
    local, reference, _, _ = windows()
    with pytest.raises(ValueError, match='level'):
        calibrate_planar_imu(local, reference,
            reference_to_body=Pose(roll=5,wrd_frame='body',ego_frame='reference'))
    cal = calibrate_planar_imu(local, reference, reference_to_body=mount).to_dict()
    cal['vertical_axis'] = [0,0,0]
    with pytest.raises(ValueError, match='invalid'):
        PlanarIMUCalibration.from_dict(cal)
