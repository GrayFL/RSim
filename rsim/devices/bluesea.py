"""Connection-only view of a local BlueSea lidar provider."""

from pathlib import Path

from rsim.runtime.host import SharedSensor


def BlueSea(port, *, history=8, transport=None):
    resolved = str(Path(port).expanduser().resolve())
    source = SharedSensor(
        key="bluesea:" + resolved, version="bluesea-v1",
        history=history, transport=transport, output_name="scan",
    )
    source.scan = source.output
    return source
