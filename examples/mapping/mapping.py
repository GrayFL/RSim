"""Read-only RGB mapping capture. Supply a local YAML recipe; no motion commands."""
import argparse
import asyncio
import json
from pathlib import Path
import time

import numpy as np

from rsim import Runtime, load_mapper, GraphMap

ASSETS = Path(__file__).resolve().parents[2] / 'assets' / 'mapping'


async def capture(config, *, seconds=15, output=None):
    output = Path(output
                 ) if output is not None else ASSETS / time.strftime(
                        'session-%Y%m%d-%H%M%S'
                     )
    output = output.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    mapper = load_mapper(
        config, overrides={'database': str(output / 'rtabmap.db')}
        )
    samples = []
    async with Runtime(mapper):
        pose = await mapper.pose.get(timeout=60)
        cloud = await mapper.rgb_map.get(timeout=30)
        initial = time.monotonic()
        while time.monotonic() - initial < seconds:
            pose = await mapper.pose.get(after=pose.sequence, timeout=5)
            samples.append([
                pose.stamp_ns, *pose.data.position, *pose.data.quat
                ])
            await asyncio.sleep(.1)
        map_frame = await mapper.map.get(timeout=30)
        view = GraphMap().update(map_frame.data)
        cloud = await mapper.rgb_map.get(
            timestamp_ns=map_frame.stamp_ns, clock=map_frame.clock
            )
        # The sampled pose may have aged out of the small default history while
        # the notebook event loop was busy; query a freshly observed frame.
        pose = await mapper.pose.get()
        assert await mapper.pose.get(
            timestamp_ns=pose.stamp_ns, clock=pose.clock
            ) is pose
        ply = await mapper.save(output / 'rgb-map.ply', frame=cloud)
        await asyncio.to_thread(view.save, output / 'graphmap')
        np.save(output / 'rgb-map.npy', cloud.data.points)
        np.save(output / 'poses.npy', np.array(samples))
        report = dict(
            points=len(cloud.data.points),
            colored_points=int(np.count_nonzero(cloud.data.points['color_valid'])),
            source_observations=len(view.source_ids),
            map_revision=view.revision,
            map_session=map_frame.data['session_id'],
            graphmap=str(output / 'graphmap'),
            geometry_source='super_lio_deskewed_laser',
            camera_depth_used=False,
            map_frame=cloud.data.frame_id,
            pose=dict(
                position=pose.data.position.tolist(),
                quaternion=pose.data.quat.tolist(),
                parent=pose.data.wrd_frame,
                child=pose.data.ego_frame
                ),
            status=(await mapper.status.get()).data,
            worker_pid=mapper.manifest['pid'],
            readonly=not cloud.data.points.flags.writeable,
            mmap=isinstance(cloud.data.points, np.memmap),
            history_lookup=True,
            ply=str(ply),
            output=str(output)
            )
        (output / 'capture.json').write_text(json.dumps(report, indent=2))
    return report


def plot_capture(output):
    """A small scientific preview; full geometry remains in the exported PLY."""
    import os
    from scipykit.mtp_initializer import subplots, disp, plt
    output = Path(output).resolve()
    os.environ['NOTEBOOK_ASSETS_ROOT'] = str(ASSETS.parent)
    os.environ['NOTEBOOK_NAME'] = 'mapping'
    cloud = np.load(output / 'rgb-map.npy')
    cloud = cloud[::max(1, len(cloud) // 50000)]
    colors = np.column_stack([cloud[key] for key in 'rgb']) / 255.
    fig, ax = subplots(figsize=(9, 7), subplot_kw={'projection': '3d'})
    ax.scatter(cloud['x'], cloud['y'], cloud['z'], c=colors, s=2)
    ax.set(xlabel='X (m)', ylabel='Y (m)', zlabel='Z (m)', title='Laser geometry with RGB colors')
    ax.view_init(elev=0, azim=180)
    disp(fig, 'rgb-map')
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('config', type=Path)
    parser.add_argument('--seconds', type=float, default=15)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--plot', action='store_true')
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error('--seconds must be positive')
    report = asyncio.run(
        capture(args.config, seconds=args.seconds, output=args.output)
        )
    print(json.dumps(report, indent=2))
    if args.plot:
        plot_capture(report['output'])


if __name__ == '__main__':
    main()
