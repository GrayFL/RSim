import asyncio
import math
import time

import numpy as np
import pytest

pytest.importorskip("graphmap.pose")
from graphmap.pose import Pose

from rsim import Component, ComponentError, Runtime, ProcessPlacement, VelocityCommand
from rsim.motion import ChassisController, MotionError
from rsim.odometry import PlanarEKF, PlanarOdometry, wrap_angle
from examples.chassis_motion import SimulatedChassis, odom_message, imu_message


def test_ekf_reduces_noisy_pose_error_and_estimates_gyro_bias():
    rng = np.random.default_rng(91)
    ekf = PlanarEKF(acceleration_std=.2, angular_acceleration_std=.3, bias_walk_std=.0005)
    covariance = np.diag([.04**2, .04**2, .03**2])
    ekf.initialize(np.zeros(3), covariance, 0)
    truth = np.zeros(3)
    raw, filtered = [], []
    for step in range(1, 1001):
        dt, velocity, rate = .02, .2, .4
        truth += [velocity * math.cos(truth[2] + rate * dt / 2) * dt,
                  velocity * math.sin(truth[2] + rate * dt / 2) * dt, rate * dt]
        ekf.predict(step * 20_000_000)
        ekf.gyroscope(rate + .035 + rng.normal(0, .01), .01**2)
        if step % 5 == 0:
            observation = truth + rng.normal(size=3) * [.04, .04, .03]
            observation[2] = wrap_angle(observation[2])
            ekf.odometry(observation, covariance)
            if step > 200:
                raw.append(observation[:2] - truth[:2])
                filtered.append(ekf.x[:2] - truth[:2])
        assert np.linalg.eigvalsh(ekf.P).min() >= -1e-12
        assert np.isfinite(ekf.x).all()
    assert np.sqrt(np.mean(np.square(filtered))) < .8 * np.sqrt(np.mean(np.square(raw)))
    assert abs(ekf.x[5] - .035) < .015
    assert abs(wrap_angle(ekf.x[2] - truth[2])) < .06


def test_ekf_clock_extrinsics_history_and_stale_stream():
    async def run():
        source = Component()
        odom = source.signal("odom", clock="robot")
        imu = source.signal("imu", clock="robot")
        other = source.signal("other", clock="other")
        with pytest.raises(ValueError, match="clock"):
            PlanarOdometry(odom, other)
        # IMU X maps onto body Z: a nonidentity mounting rotation must be used.
        transform = Pose(pitch=-90, wrd_frame="body", ego_frame="imu")
        fused = PlanarOdometry(odom, imu, T_body_imu=transform)
        async with Runtime(fused):
            data = imu_message(0, frame="imu")
            data["angular_velocity"]["x"] = .3
            await imu.publish(data, stamp_ns=10, clock="robot")
            await odom.publish(odom_message(Pose(wrd_frame="odom", ego_frame="body")),
                               stamp_ns=10, clock="robot")
            frame = await fused.pose.get(timeout=1)
            assert isinstance(frame.data, Pose)
            assert (frame.data.wrd_frame, frame.data.ego_frame) == ("odom", "body")
            assert (await fused.pose.get(timestamp_ns=10, clock="robot")) is frame
            assert fused.filter.x[4] > .2
            # Duplicate stamps do not masquerade as fresh poses.
            await imu.publish(data, stamp_ns=10, clock="robot")
            await odom.publish(odom_message(frame.data), stamp_ns=10, clock="robot")
            with pytest.raises(TimeoutError):
                await fused.pose.get(after=frame.sequence, timeout=.05)
            assert fused.dropped >= 2
            # Odom alone cannot keep the fused output fresh after IMU stops.
            await odom.publish(odom_message(frame.data), stamp_ns=100_000_010, clock="robot")
            with pytest.raises(TimeoutError):
                await fused.pose.get(after=frame.sequence, timeout=.05)
    asyncio.run(run())


def test_ekf_rejects_missing_extrinsics_and_unavailable_measurement():
    async def run():
        chassis = SimulatedChassis()
        fused = PlanarOdometry(chassis.odom, chassis.imu,
                               T_body_imu=Pose(wrd_frame="body", ego_frame="wrong"))
        async with Runtime(fused):
            with pytest.raises(ComponentError):
                await fused.pose.get(timeout=1)
        fused = PlanarOdometry(chassis.odom, chassis.imu)
        fused._body = "body"
        data = imu_message(0)
        data["angular_velocity_covariance"][0] = -1
        with pytest.raises(ValueError, match="unavailable"):
            fused._read_imu(data)
        data = imu_message(0, frame="other")
        with pytest.raises(ValueError, match="T_body_imu"):
            fused._read_imu(data)
    asyncio.run(run())


def test_forward_reverse_left_right_and_multiple_turns():
    async def run():
        chassis = SimulatedChassis(speedup=6, noise=False)
        control = ChassisController(chassis, motion_enabled=True, hz=100,
                                    max_linear=.4, max_angular=2., settle_samples=2)
        async with Runtime(control):
            start = (await control.pose.get(timeout=1)).data
            end = await control.move(.3, timeout=5)
            assert .27 < ((~start) * end).position[0] < .33
            start = end
            end = await control.move(-.2, timeout=5)
            assert -.23 < ((~start) * end).position[0] < -.17
            start_yaw = chassis.state[2]
            await control.rotate(yaw_deg=450, timeout=8)
            assert abs(chassis.state[2] - start_yaw - math.radians(450)) < .07
            start_yaw = chassis.state[2]
            await control.rotate(yaw_rad=-math.pi / 2, timeout=5)
            assert abs(chassis.state[2] - start_yaw + math.pi / 2) < .07
            assert chassis.commands[-1][1] == VelocityCommand()
        values = [item[1] for item in chassis.commands]
        assert any(item.linear_x < 0 for item in values)
        assert any(item.linear_x > 0 for item in values)
        assert any(item.angular_z < 0 for item in values)
        assert any(item.angular_z > 0 for item in values)
    asyncio.run(run())


def test_disabled_motion_zero_requests_and_angle_validation():
    async def run():
        chassis = SimulatedChassis(noise=True)
        control = ChassisController(chassis)
        async with Runtime(control):
            with pytest.raises(MotionError, match="motion_enabled"):
                await control.move(.1)
            for kwargs in ({}, {"yaw_deg": 1, "yaw_rad": 1}):
                with pytest.raises(ValueError, match="exactly one"):
                    await control.rotate(**kwargs)
            for value in (float("nan"), float("inf")):
                with pytest.raises(ValueError, match="finite"):
                    await control.move(value)
            await control.move(0, timeout=1)
            await control.rotate(yaw_rad=0, timeout=1)
        assert chassis.commands and all(command == VelocityCommand() for _, command in chassis.commands)
    asyncio.run(run())


def test_cancel_timeout_stop_concurrency_and_runtime_close_all_stop():
    async def started(control, chassis):
        cursor = len(chassis.commands)
        task = asyncio.create_task(control.move(10, timeout=10))
        async with asyncio.timeout(1):
            while not any(command.linear_x > 0 for _, command in chassis.commands[cursor:]):
                await asyncio.sleep(.005)
        return task

    async def run():
        chassis = SimulatedChassis(noise=False)
        control = ChassisController(chassis, motion_enabled=True)
        async with Runtime(control):
            task = await started(control, chassis)
            with pytest.raises(MotionError, match="active"):
                await control.rotate(90)
            task.cancel()
            with pytest.raises(asyncio.CancelledError):
                await task
            assert chassis.commands[-1][1] == VelocityCommand()
            with pytest.raises(TimeoutError):
                await control.move(10, timeout=.08)
            assert chassis.commands[-1][1] == VelocityCommand()
            task = await started(control, chassis)
            await control.stop()
            with pytest.raises(MotionError, match="interrupted"):
                await task
            assert chassis.commands[-1][1] == VelocityCommand()
            task = await started(control, chassis)
        with pytest.raises(MotionError, match="closed"):
            await task
        assert chassis.commands[-1][1] == VelocityCommand()
    asyncio.run(run())


def test_stale_pose_stops_instead_of_driving_on_cached_feedback():
    async def run():
        source = Component()
        pose = source.signal("pose")
        chassis = SimulatedChassis(noise=False)
        control = ChassisController(pose=pose, velocity=chassis.velocity,
                                    motion_enabled=True, pose_timeout=.07)
        async with Runtime(control):
            await pose.publish(Pose(wrd_frame="odom", ego_frame="body"), stamp_ns=0, clock="test")
            with pytest.raises(MotionError, match="stale"):
                await control.move(1, timeout=1)
            assert chassis.commands[-1][1] == VelocityCommand()
    asyncio.run(run())


def test_pose_codec_and_ekf_process_placement(tmp_path):
    from rsim.shared import SharedStore, decode
    from rsim.model import Frame
    original = Pose(x=1, y=2, yaw=120, wrd_frame="odom", ego_frame="body")
    store = SharedStore(tmp_path / "pose")
    try:
        descriptor = store.put(Frame(original, 123, "test", time.time_ns(), 1))
        restored = decode(descriptor["data"], store.directory)
        assert isinstance(restored, Pose) and original.allclose(restored)
        assert (restored.wrd_frame, restored.ego_frame) == ("odom", "body")
    finally:
        store.close()

    async def run():
        chassis = SimulatedChassis(noise=False)
        control = ChassisController(chassis)
        async with Runtime(control, placement={control.odometry: ProcessPlacement("ekf")}):
            frame = await control.pose.get(timeout=15)
            assert isinstance(frame.data, Pose)
            estimate = await control.odometry.estimate.get(timeout=2)
            assert isinstance(estimate.data["covariance"], np.memmap)
            await control.move(0, timeout=2)
        assert all(command == VelocityCommand() for _, command in chassis.commands)
    asyncio.run(run())
