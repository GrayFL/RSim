"""Record ordinary RSim sensor ports without importing MuJoCo or its driver."""
import asyncio
import json
from pathlib import Path

import numpy as np


async def snapshot(source, directory):
    # Match RGBD by its simulation clock, independently of reader sequence IDs.
    image = await source.image.get(timeout=15)
    async def matching(port):
        async with asyncio.timeout(5):
            frame = await port.get()
            while frame.stamp_ns < image.stamp_ns:
                frame = await port.get(after=frame.sequence)
            return await port.get(timestamp_ns=image.stamp_ns, clock=image.clock)
    depth, info = await asyncio.gather(matching(source.depth), matching(source.camera_info))
    scan = await source.scan.get(timeout=5)
    def save():
        from PIL import Image
        output = Path(directory)
        output.mkdir(parents=True, exist_ok=True)
        Image.fromarray(image.data.pixels).save(output / 'rgb.png')
        np.save(output / 'depth_m.npy', depth.data.pixels)
        angles = scan.data['angle_min'] + np.arange(len(scan.data['ranges']))*scan.data['angle_increment']
        np.savez_compressed(output / 'sensors.npz', time=image.stamp_ns/1e9,
                            image_stamp_ns=image.stamp_ns, scan_stamp_ns=scan.stamp_ns,
                            angles=angles, ranges=scan.data['ranges'], K=info.data['K'],
                            world_from_base=image.metadata['world_from_base'],
                            world_from_lidar=scan.metadata['world_from_lidar'],
                            world_from_camera_optical=info.data['world_from_camera_optical'])
        metadata = dict(clock=image.clock, image_stamp_ns=image.stamp_ns, scan_stamp_ns=scan.stamp_ns,
                        depth_unit='m', depth_type='optical_z', invalid_depth='NaN', invalid_lidar='+inf',
                        K=info.data['K'].tolist(), world_from_base=image.metadata['world_from_base'].tolist(),
                        world_from_lidar=scan.metadata['world_from_lidar'].tolist(),
                        world_from_camera_optical=info.data['world_from_camera_optical'].tolist())
        (output / 'metadata.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')
    await asyncio.to_thread(save)
    return dict(image_shape=list(image.data.pixels.shape), depth_shape=list(depth.data.pixels.shape),
                lidar_beams=len(scan.data['ranges']), shared_image=isinstance(image.data.pixels, np.memmap))
