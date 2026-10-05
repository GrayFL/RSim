import numpy as np
from rsim.core.model import Image, PointCloud

def pointcloud_array(msg):
    types = {
        1: "i1",
        2: "u1",
        3: "i2",
        4: "u2",
        5: "i4",
        6: "u4",
        7: "f4",
        8: "f8"
        }
    endian = ">" if msg.is_bigendian else "<"
    names, formats, offsets = [], [], []
    for field in msg.fields:
        dtype = np.dtype(endian + types[field.datatype])
        names.append(field.name)
        formats.append(
            dtype if field.count == 1 else (dtype, (field.count, ))
            )
        offsets.append(field.offset)
    dtype = np.dtype({
        "names": names,
        "formats": formats,
        "offsets": offsets,
        "itemsize": msg.point_step
        })
    result = np.ndarray((msg.height, msg.width),
                        dtype=dtype,
                        buffer=msg.data,
                        strides=(msg.row_step, msg.point_step))
    result.flags.writeable = False
    return PointCloud(result, msg.header.frame_id)


def image_array(msg):
    formats = {
        "rgb8": ("u1", 3),
        "bgr8": ("u1", 3),
        "rgba8": ("u1", 4),
        "bgra8": ("u1", 4),
        "mono8": ("u1", 1),
        "8UC1": ("u1", 1),
        "mono16": ("u2", 1),
        "16UC1": ("u2", 1),
        "32FC1": ("f4", 1)
        }
    if msg.encoding not in formats:
        raise ValueError(f"unsupported image encoding: {msg.encoding}")
    scalar, channels = formats[msg.encoding]
    dtype = np.dtype((">" if msg.is_bigendian else "<") + scalar)
    shape = (msg.height, msg.width) + ((channels, ) if channels > 1 else
                                        ())
    strides = (msg.step, channels *
                dtype.itemsize) + ((dtype.itemsize, ) if channels > 1 else
                                    ())
    result = np.ndarray(
        shape, dtype=dtype, buffer=msg.data, strides=strides
        )
    result.flags.writeable = False
    return Image(result, msg.encoding, msg.header.frame_id)


def imu_data(msg):
    data = {"header": {"frame_id": msg.header.frame_id}}
    for name, axes in (("orientation", "xyzw"), ("angular_velocity", "xyz"),
                       ("linear_acceleration", "xyz")):
        data[name] = {axis: float(getattr(getattr(msg, name), axis)) for axis in axes}
        data[name + "_covariance"] = list(getattr(msg, name + "_covariance"))
    return data
