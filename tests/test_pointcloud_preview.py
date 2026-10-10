import struct

import pytest

pytest.importorskip('sensor_msgs.msg')
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from rsim.apps.pointcloud_preview import decimate


def test_preview_bounds_and_preserves_organized_cloud_records():
    # Two rows of five points, padded with bytes that must never become data.
    records = [struct.pack('<ff', i, 100 + i) for i in range(10)]
    raw = b''.join(records[:5]) + b'padding!' + b''.join(records[5:]) + b'padding!'
    message = PointCloud2(header=Header(frame_id='lidar'), height=2, width=5,
        fields=[PointField(name='x', offset=0, datatype=7, count=1),
                PointField(name='intensity', offset=4, datatype=7, count=1)],
        point_step=8, row_step=48, data=raw)
    message.header.stamp.sec = 123
    result = decimate(message, max_points=8, max_bytes=32)
    assert bytes(result.data) == b''.join(records[i] for i in (0, 3, 6, 9))
    assert result.height == 1 and result.width == 4 and result.row_step == 32
    assert result.header == message.header and result.fields == message.fields
    assert result.is_bigendian == message.is_bigendian
    assert bytes(message.data) == raw
    message.row_step = 39
    with pytest.raises(ValueError, match='layout'):
        decimate(message)


def test_preview_empty_cloud_and_record_exceeding_budget():
    message = PointCloud2(height=1, width=0, point_step=16, row_step=0, data=b'')
    assert decimate(message).width == 0
    with pytest.raises(ValueError, match='one point'):
        decimate(message, max_bytes=8)
