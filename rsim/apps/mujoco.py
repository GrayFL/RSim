from __future__ import annotations

import argparse
import asyncio
import uuid
from contextlib import nullcontext
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import threading
import time

import numpy as np

from rsim.runtime import Runtime
from rsim.core import VelocityCommand
from rsim.devices import Chassis
from rsim.drivers import MuJoCo, Chassis as ChassisProvider
from rsim.transport.descriptor import TransportConfig
from rsim.adapters.mujoco.config import SensorConfig
from rsim.adapters.mujoco.scene import prepare_scene


class Keyboard:
    def __init__(self):
        self.command = (0.0, 0.0)
        self.reset = self.snapshot = self.quit = False
        self.lock = threading.Lock()

    def __call__(self, key):
        with self.lock:
            commands = {ord("W"): (.35, 0), ord("S"): (-.25, 0),
                        ord("A"): (0, .8), ord("D"): (0, -.8), ord(" "): (0, 0)}
            if key in commands:
                self.command = commands[key]
            if key == ord("R"):
                self.reset = True
                self.command = (0, 0)
            if key == ord("P"):
                self.snapshot = True
            if key == ord("Q"):
                self.quit = True


def demo_velocity(sim, scan):
    """Reactive demo with ground-truth floor/clearance guard (not SLAM)."""
    x, y, yaw = sim.pose2d
    forward = np.array([np.cos(yaw), np.sin(yaw)])
    safe = min(sim.clearance_at(np.array([x, y]) + forward*d) for d in (0.2, 0.4, 0.6)) > .54
    front = scan.ranges[np.abs(scan.angles) < np.deg2rad(35)].min()
    if not safe or front < .85:
        return 0.0, .65
    return .30, 0.0


def preview_sensors(frame, scan):
    import cv2
    rgb = cv2.cvtColor(frame.rgb, cv2.COLOR_RGB2BGR)
    depth = cv2.applyColorMap(np.uint8(np.nan_to_num(frame.depth, nan=0).clip(0, 10)*25.5), cv2.COLORMAP_TURBO)
    depth[~np.isfinite(frame.depth)] = 0
    display = np.concatenate([rgb, depth], axis=1)
    cv2.putText(display, f"RGB / depth [0..10m]  t={frame.time:.2f}s", (12, 25), cv2.FONT_HERSHEY_SIMPLEX, .65, (255, 255, 255), 2)
    cv2.imshow("RGBD sensors", display)
    canvas = np.full((480, 480, 3), 245, np.uint8)
    points = scan.points()
    pixels = np.round(np.array([240, 240]) + np.column_stack((-points[:, 1], -points[:, 0]))*30).astype(int)
    for u, v in pixels:
        if 0 <= u < 480 and 0 <= v < 480:
            cv2.circle(canvas, (u, v), 2, (190, 80, 20), -1)
    cv2.rectangle(canvas, (231, 231), (249, 249), (20, 20, 220), 1)
    cv2.arrowedLine(canvas, (240, 240), (240, 215), (20, 20, 220), 2)
    cv2.imshow("2D lidar: forward up, left left", canvas)
    return cv2.waitKey(1) & 0xff


def parser(default_scene=None, default_cache='assets/mujoco/scenes', default_output='assets/mujoco/latest'):
    p = argparse.ArgumentParser(description="0.6 m differential-drive robot in a textured GLB scene")
    p.add_argument("--scene", default=default_scene, required=default_scene is None, help="GLB/GLTF, composition JSON, or prepared scene directory")
    p.add_argument("--up-axis", choices=["auto", "y", "z"], default="auto")
    p.add_argument("--floor-height", type=float, help="Selected floor height along the source up axis")
    p.add_argument("--resolution", type=float, default=.08, help="Collision surface voxel size [m]")
    p.add_argument("--cache-dir", default=default_cache)
    p.add_argument("--rebuild", action="store_true")
    p.add_argument("--prepare-only", action="store_true")
    p.add_argument("--spawn", nargs=3, type=float, metavar=("X", "Y", "YAW"), help="World x/y [m], yaw [rad]")
    p.add_argument("--headless", action="store_true", help="Run without interactive windows; final RGBD still uses OpenGL")
    p.add_argument("--mode", choices=["teleop", "demo", "idle"], default="teleop")
    p.add_argument("--seconds", type=float, help="Simulated duration; default: GUI unlimited, headless 10 s")
    p.add_argument("--output", default=default_output)
    p.add_argument("--preview", action="store_true", help="Live RGBD + lidar windows (requires cv2)")
    p.add_argument("--record-every", type=float, default=0, help="Save RGBD/lidar every N simulated seconds; 0 = final only")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    return p


def save_run(sim, output, trajectory, scans, scan_poses, elapsed, mode):
    """Simulation diagnostics run on the physics/renderer owner thread."""
    scan, frame = sim.save_snapshot(output)
    from PIL import Image
    from rsim.apps.mujoco_plotting import diagnostic
    Image.fromarray(sim.overview()).save(output / 'robot_overview.png')
    np.savetxt(output / 'trajectory.csv', trajectory, delimiter=',',
               header='time,x,y,yaw,command_v,command_omega', comments='')
    np.savez_compressed(output / 'lidar_history.npz', samples=np.asarray(scans),
                        angles=scan.angles, world_from_lidar=np.asarray(scan_poses))
    diagnostic(sim, scan, frame, trajectory, output / 'diagnostic.png')
    summary = dict(backend='rsim.drivers.MuJoCo', control='rsim.devices.Chassis',
                   simulated_seconds=float(sim.data.time), wall_seconds=elapsed,
                   physics_steps=round(sim.data.time/sim.robot.timestep),
                   final_pose=sim.pose2d.tolist(),
                   valid_lidar_beams=int(np.isfinite(scan.ranges).sum()),
                   depth_valid_fraction=float(np.isfinite(frame.depth).mean()), mode=mode)
    (output / 'run_summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    return summary


async def run(args, scene, preview_executor=None):
    source = MuJoCo(scene_dir=scene, spawn=args.spawn,
                    sensors=SensorConfig(width=args.width, height=args.height), motion_enabled=True)
    transport = TransportConfig('inprocess')
    name = 'mujoco_' + uuid.uuid4().hex
    provider = ChassisProvider(name=name, pose=source.pose, velocity=source.velocity,
                               state=source.state, motion_enabled=True, transport=transport,
                               control=dict(max_linear=.6, max_angular=1.2))
    chassis = Chassis(name, transport=transport)
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    keys = Keyboard()
    duration = args.seconds if args.seconds is not None else (10. if args.headless else np.inf)
    async def preview(frame, scan):
        return await asyncio.get_running_loop().run_in_executor(
            preview_executor, preview_sensors, frame, scan)
    async with Runtime(provider, chassis, source.scan):
        print(f'Loaded {source.simulation.model.ngeom} geoms; control=rsim.devices.Chassis', flush=True)
        viewer_context = nullcontext(None)
        if not args.headless:
            import mujoco.viewer
            viewer_context = mujoco.viewer.launch_passive(source.simulation.model, source.simulation.data,
                                                          key_callback=keys)
            print('W/S forward/back, A/D turn, Space stop, R reset, P snapshot, Q quit.', flush=True)
        trajectory, scans, scan_poses = [], [], []
        state_cursor = scan_cursor = 0
        next_preview = next_record = 0.
        started = time.perf_counter()
        with viewer_context as viewer:
            if viewer is not None:
                with viewer.lock():
                    viewer.opt.geomgroup[:] = [1, 1, 0, 0, 0, 0]
                    viewer.cam.lookat[:] = [*source.simulation.pose2d[:2], .4]
                    viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 4., 135, -55
            # Window/GL startup can take longer than the controller's freshness
            # limit. Warm them with no live motion lease, then wait for physics.
            if args.preview:
                frame, scan = await source.inspect(lambda sim: (sim.rgbd(), sim.lidar()))
                await preview(frame, scan)
            fresh = await source.state.get()
            await source.state.get(after=fresh.sequence, timeout=5)
            try:
                while True:
                    state = await source.state.get(after=state_cursor, timeout=5)
                    state_cursor = state.sequence
                    stamp = state.data['physics_time']
                    pose = (await source.pose.get()).data
                    with keys.lock:
                        command, reset, save, quit_requested = keys.command, keys.reset, keys.snapshot, keys.quit
                        keys.reset = keys.snapshot = False
                    if stamp >= duration or quit_requested or (viewer is not None and not viewer.is_running()):
                        break
                    if reset:
                        await chassis.stop()
                        await source.reset(args.spawn)
                        trajectory.clear()
                        scans.clear()
                        scan_poses.clear()
                        scan_cursor = (await source.scan.get()).sequence - 1
                        next_preview = next_record = 0.
                        continue
                    for sample in source.scan.frames:
                        if sample.sequence > scan_cursor:
                            scans.append(np.r_[sample.stamp_ns/1e9, sample.data['ranges']])
                            scan_poses.append(sample.metadata['world_from_lidar'])
                            scan_cursor = sample.sequence
                    if args.mode == 'demo':
                        command = await source.inspect(lambda sim: demo_velocity(sim, sim.lidar()))
                    elif args.mode == 'idle':
                        command = (0., 0.)
                    await chassis.drive(VelocityCommand(*command), ttl=.3)
                    trajectory.append([state.stamp_ns/1e9, *pose.position[:2], pose.euler_rad[2], *command])
                    if save or (args.record_every and stamp >= next_record):
                        await source.inspect(lambda sim: sim.save_snapshot(output / f'frame_{state_cursor:08d}'))
                        next_record = stamp + args.record_every
                    if args.preview and stamp >= next_preview:
                        frame, scan = await source.inspect(lambda sim: (sim.rgbd(), sim.lidar()))
                        key = await preview(frame, scan)
                        if key < 128:
                            keys(ord(chr(key).upper()))
                        next_preview = stamp + .2
                    if viewer is not None:
                        await source.inspect(lambda sim: viewer.sync())
            finally:
                await chassis.stop()
        elapsed = time.perf_counter()-started
        summary = await source.inspect(save_run, output, trajectory, scans, scan_poses, elapsed, args.mode)
        summary['scene_cache'] = str(scene)
        (output / 'run_summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
        print(json.dumps(summary, indent=2), flush=True)
        print(f'Saved RGBD, lidar, transforms, trajectory and diagnostics to {output.resolve()}', flush=True)


def main(argv=None, *, default_scene=None, default_cache='assets/mujoco/scenes',
         default_output='assets/mujoco/latest'):
    p = parser(default_scene, default_cache, default_output)
    args = p.parse_args(argv)
    if args.seconds is not None and (not np.isfinite(args.seconds) or args.seconds <= 0):
        p.error('--seconds must be finite and positive')
    if not np.isfinite(args.record_every) or args.record_every < 0:
        p.error('--record-every must be finite and nonnegative')
    if args.preview and args.headless:
        p.error('--preview requires an interactive run')
    scene = prepare_scene(args.scene, args.cache_dir, up_axis=args.up_axis,
                          floor_height=args.floor_height, pitch=args.resolution, force=args.rebuild)
    if args.prepare_only:
        print(scene)
        return
    def close_preview():
        import cv2
        cv2.destroyAllWindows()
    with ThreadPoolExecutor(max_workers=1, thread_name_prefix='rsim-preview') as preview_executor:
        try:
            asyncio.run(run(args, scene, preview_executor))
        except KeyboardInterrupt:
            pass
        finally:
            if args.preview:
                preview_executor.submit(close_preview).result()


if __name__ == '__main__':
    main()


