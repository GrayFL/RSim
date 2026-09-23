from array import array
from types import SimpleNamespace as NS

import numpy as np
import pytest

from rsim.ros import image_array, pointcloud_array


def test_pointcloud_padding_and_endianness_are_preserved():
    raw = bytearray(64)
    expected = np.ndarray((2, 2), dtype=">f4", buffer=raw, strides=(32, 12))
    expected[:] = [[1, 2], [3, 4]]
    msg = NS(height=2, width=2, is_bigendian=True, point_step=12, row_step=32,
             data=raw, fields=[NS(name="x", datatype=7, count=1, offset=0)],
             header=NS(frame_id="lidar"))
    cloud = pointcloud_array(msg)
    np.testing.assert_array_equal(cloud.points["x"], [[1, 2], [3, 4]])
    assert np.shares_memory(cloud.points, expected)
    assert not cloud.points.flags.writeable


def test_image_padding_channels_and_no_copy():
    raw = array("B", range(16))
    image = image_array(NS(encoding="rgb8", height=2, width=2, step=8, data=raw,
                           is_bigendian=False, header=NS(frame_id="camera")))
    assert image.pixels.shape == (2, 2, 3)
    np.testing.assert_array_equal(image.pixels[1, 0], [8, 9, 10])
    assert np.shares_memory(image.pixels, np.frombuffer(raw, dtype="u1"))
    with pytest.raises(ValueError):
        image.pixels[0, 0, 0] = 4
