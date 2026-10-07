"""ROS-free views of a shared mapping provider."""
import asyncio
from pathlib import Path

from rsim.core import Component
from rsim.runtime.host import SharedSensor


class MappingView(Component):
    def __init__(self, source):
        super().__init__(inputs=(source.output,))
        self.source = source
        self.previous = 0
        self.identities = {}
        for name in ('pose', 'odometry', 'rgb_map', 'map', 'status'):
            setattr(self, name, self.signal(name, history=source.output.history_size))

    async def open(self):
        self.previous = 0
        self.identities.clear()
        self.task('mapping-ports', self.receive, hz=100)

    async def receive(self):
        snapshot = await self.source.get(after=self.previous)
        self.previous = snapshot.sequence
        for name, frame in snapshot.data.items():
            identity = (frame.stamp_ns, frame.received_ns, frame.sequence)
            if identity == self.identities.get(name):
                continue
            await self.outputs[name].publish(frame.data, stamp_ns=frame.stamp_ns,
                clock=frame.clock, received_ns=frame.received_ns)
            self.identities[name] = identity

    async def save(self, path, *, frame=None, timeout=30):
        """Export the latest (or a retained) RGB cloud as binary PLY."""
        frame = await self.rgb_map.get(timeout=timeout) if frame is None else frame
        path = Path(path).expanduser().resolve()
        if path.suffix.lower() != '.ply':
            raise ValueError('RGB map export expects a .ply path')
        def write():
            path.parent.mkdir(parents=True, exist_ok=True)
            header = ('ply\nformat binary_little_endian 1.0\n'
                      f'comment frame {frame.data.frame_id}\nelement vertex {len(frame.data.points)}\n'
                      'property float x\nproperty float y\nproperty float z\n'
                      'property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n')
            with path.open('wb') as file:
                file.write(header.encode('ascii'))
                # PLY geometry/color remain widely readable; stable IDs and
                # provenance live in the lossless map snapshot/graphmap archive.
                points = frame.data.points
                import numpy as np
                packed = np.empty(len(points), dtype=[(key, '<f4') for key in 'xyz'] + [(key, 'u1') for key in 'rgb'])
                for key in packed.dtype.names:
                    packed[key] = points[key]
                packed.tofile(file)
            return path
        return await asyncio.to_thread(write)


def Mapper(name='mapping', *, history=3, transport=None):
    """Connect to an existing provider; exposes pose, rgb_map and provenance map."""
    return MappingView(SharedSensor(key='mapper:' + name, version='mapping-v2',
                                   history=history, transport=transport))
