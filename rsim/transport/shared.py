"""Immutable tmpfs arrays. DDS carries descriptors, never the array payload.

Each committed sample has a fresh inode. Unlinking evicted files does not
invalidate existing read-only mmap views: the kernel holds their pages alive.
"""
from __future__ import annotations

from collections import deque
from contextvars import ContextVar
from pathlib import Path
import os
import math
import shutil
import uuid
import weakref
import sys
from dataclasses import fields, replace

import numpy as np


current_store = ContextVar("rsim_shared_store", default=None)


def _records():
    # Closed schema: a descriptor cannot import or instantiate arbitrary code.
    from rsim.core.model import Frame
    from rsim.core.commands import CommandEnvelope, VelocityCommand
    return {cls.__name__: cls for cls in (Frame, CommandEnvelope, VelocityCommand)}


def _is_pose(data):
    # Keep graphmap optional for camera/lidar-only clients. Its class must
    # already be loaded for an actual Pose instance to exist.
    module = sys.modules.get("graphmap.pose")
    return module is not None and type(data) is module.Pose


def allocate(shape, dtype=np.float64):
    """Allocate output directly in the worker's shared store, or locally outside it.

    Publishing transfers ownership: writers must not mutate published arrays or
    retain writable aliases. Readers receive separate read-only OS mappings.
    """
    store = current_store.get()
    return np.empty(shape, dtype=dtype) if store is None else store.allocate(shape, dtype)


def _unlink(path):
    Path(path).unlink(missing_ok=True)


class SharedStore:
    def __init__(self, directory, *, history=16, reuse=False):
        self.directory = Path(directory)
        self.directory.mkdir(mode=0o700, parents=True, exist_ok=True)
        self.history = history
        self._committed = deque()
        self._reuse = reuse
        self._prepared = {}

    def prepare(self, data, memo=None):
        """Make retained worker history refer to shared arrays from publication.

        Ordinary producer buffers only coexist during this ingress copy, rather
        than leaving a second payload copy in the source's bounded history.
        """
        from rsim.core.model import PointCloud, Image
        memo = {} if memo is None else memo
        if isinstance(data, np.ndarray):
            if id(data) in memo:
                return memo[id(data)]
            if isinstance(data, np.memmap):
                data.flags.writeable = False
                return data
            cached = self._prepared.get(id(data)) if self._reuse else None
            if cached is not None and cached[0]() is data:
                return cached[1]
            result = self.allocate(data.shape, data.dtype)
            np.copyto(result, data)
            result.flags.writeable = False
            memo[id(data)] = result
            if self._reuse:
                identity = id(data)
                self._prepared[identity] = (weakref.ref(data, lambda _: self._prepared.pop(identity, None)),
                                            result)
            return result
        if isinstance(data, PointCloud):
            return PointCloud(self.prepare(data.points, memo), data.frame_id)
        if isinstance(data, Image):
            return Image(self.prepare(data.pixels, memo), data.encoding, data.frame_id)
        if type(data) in _records().values():
            return replace(data, **{field.name: self.prepare(getattr(data, field.name), memo)
                                    for field in fields(data)})
        if isinstance(data, dict):
            return {k: self.prepare(v, memo) for k, v in data.items()}
        if isinstance(data, (tuple, list)):
            values = [self.prepare(v, memo) for v in data]
            return tuple(values) if isinstance(data, tuple) else values
        return data

    def put(self, frame):
        directory = self.directory / uuid.uuid4().hex
        directory.mkdir(mode=0o700)
        try:
            descriptor = self._encode(frame.data, directory)
        except BaseException:
            shutil.rmtree(directory)
            raise
        self._committed.append(directory)
        while len(self._committed) > self.history:
            shutil.rmtree(self._committed.popleft())
        return {"data": descriptor, "stamp_ns": frame.stamp_ns, "clock": frame.clock,
                "received_ns": frame.received_ns, "sequence": frame.sequence}

    def allocate(self, shape, dtype):
        if np.dtype(dtype).hasobject:
            raise TypeError("object arrays cannot be shared")
        directory = self.directory / "allocated"
        directory.mkdir(exist_ok=True)
        path = directory / (uuid.uuid4().hex + ".npy")
        array = np.lib.format.open_memmap(path, mode="w+", dtype=dtype, shape=shape)
        array._rsim_owned = True
        weakref.finalize(array, _unlink, str(path))
        return array

    def _encode(self, data, directory):
        from rsim.core.model import PointCloud, Image
        if _is_pose(data):
            return {"type": "graphmap_pose", "translation": data.position.tolist(),
                    "quaternion": data.quat.tolist(), "scale": data.scale,
                    "wrd_frame": data.wrd_frame, "ego_frame": data.ego_frame}
        if isinstance(data, np.ndarray):
            if data.dtype.hasobject:
                raise TypeError("object arrays cannot be shared")
            path = directory / (uuid.uuid4().hex + ".npy")
            linked = False
            if isinstance(data, np.memmap) and data.filename and Path(data.filename).exists():
                original = np.load(data.filename, mmap_mode="r", allow_pickle=False)
                if getattr(data, "_rsim_owned", False):
                    data.flags.writeable = False
                # Views can retain memmap.offset even when their first element
                # moves. Only reuse the complete array's backing mapping.
                if (data.base is data._mmap and data.shape == original.shape
                        and data.dtype == original.dtype and data.strides == original.strides
                        and data.offset == original.offset and not data.flags.writeable):
                    os.link(data.filename, path)
                    linked = True
            if not linked:
                with path.open("wb") as file:
                    np.save(file, data, allow_pickle=False)
            return {"type": "array", "path": str(path)}
        if isinstance(data, PointCloud):
            return {"type": "points", "points": self._encode(data.points, directory),
                    "frame_id": data.frame_id}
        if isinstance(data, Image):
            return {"type": "image", "pixels": self._encode(data.pixels, directory),
                    "encoding": data.encoding, "frame_id": data.frame_id}
        if type(data) in _records().values():
            return {"type": "record", "name": type(data).__name__,
                    "fields": {field.name: self._encode(getattr(data, field.name), directory)
                               for field in fields(data)}}
        if isinstance(data, dict):
            if not all(isinstance(k, str) for k in data):
                raise TypeError("shared dictionary keys must be strings")
            return {"type": "dict", "items": {k: self._encode(v, directory) for k, v in data.items()}}
        if isinstance(data, (tuple, list)):
            return {"type": "tuple" if isinstance(data, tuple) else "list",
                    "items": [self._encode(v, directory) for v in data]}
        if isinstance(data, float) and not math.isfinite(data):
            return {"type": "nonfinite", "value": str(data)}
        if data is None or isinstance(data, (str, bool, int, float)):
            return {"type": "scalar", "value": data}
        raise TypeError(f"unsupported shared data type: {type(data)}")

    def close(self):
        self._prepared.clear()
        shutil.rmtree(self.directory, ignore_errors=True)


def decode(data, allowed_directory):
    from rsim.core.model import PointCloud, Image
    kind = data["type"]
    if kind == "graphmap_pose":
        from graphmap.pose import Pose
        return Pose(position=data["translation"], rotation=data["quaternion"],
                    scale=data["scale"], wrd_frame=data["wrd_frame"], ego_frame=data["ego_frame"])
    if kind == "array":
        path = Path(data["path"]).resolve()
        if not path.is_relative_to(Path(allowed_directory).resolve()):
            raise ValueError("descriptor path is outside this sensor's store")
        return np.load(path, mmap_mode="r", allow_pickle=False)
    if kind == "points":
        return PointCloud(decode(data["points"], allowed_directory), data["frame_id"])
    if kind == "image":
        return Image(decode(data["pixels"], allowed_directory), data["encoding"], data["frame_id"])
    if kind == "dict":
        return {k: decode(v, allowed_directory) for k, v in data["items"].items()}
    if kind == "record":
        record = _records()[data["name"]]
        return record(**{key: decode(value, allowed_directory) for key, value in data["fields"].items()})
    if kind in ("tuple", "list"):
        values = [decode(v, allowed_directory) for v in data["items"]]
        return tuple(values) if kind == "tuple" else values
    if kind == "scalar":
        return data["value"]
    if kind == "nonfinite":
        return float(data["value"])
    raise ValueError(f"unknown descriptor type: {kind}")
