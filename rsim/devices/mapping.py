"""ROS-free views of a shared mapping provider."""
import asyncio
from pathlib import Path

from rsim.core import Component
from rsim.runtime.sharing import SharedComponent, PortSpec


class MappingSave:
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



def mapping_ports(history=3):
    return {name: PortSpec(schema=schema, clock='ros:system', history_capacity=history)
            for name, schema in {'pose': 'graphmap.pose.v1', 'odometry': 'graphmap.pose.v1',
                                  'map': 'rsim.laser-map.v1', 'rgb_map': 'rsim.pointcloud.v1',
                                  'status': 'rsim.mapping-status.v1'}.items()}


class MappingView(MappingSave, Component):
    """Public aliases of a native or shared multi-output mapping component."""
    def __init__(self, source):
        super().__init__(source)
        self.source = source
        for name in mapping_ports():
            self.expose(name, source.outputs[name])


class MappingClient(MappingSave, SharedComponent):
    pass


def Mapper(name='mapping', *, history=3, transport=None):
    """Connection-only view; request exactly the outputs the application needs."""
    return MappingClient(key='mapper:' + name, ports=mapping_ports(history),
                         interface_version='mapping-ports-v1', transport=transport)
