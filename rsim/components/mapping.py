"""Laser keyframes, stable observations and revisioned graphmap voxel views.

An observation is identified by (session UUID, node_id << 32 | local row).
Its identity and local geometry never depend on the optimized world pose.
"""
from dataclasses import dataclass
import json
from pathlib import Path
import uuid

import numpy as np
from graphmap.index_db import IndexDB, point_keys_from_xyz
from graphmap.infopoints import InfoPoints
from graphmap.pose import Pose


SCHEMA = 'rsim.lidar-map.v1'


@dataclass(frozen=True)
class LaserKeyframe:
    node_id: int
    stamp_ns: int
    scan_stamp_ns: int
    xyz: np.ndarray  # Deskewed laser points in base at RGB acquisition time.
    rgba: np.ndarray
    pixels: np.ndarray  # RGB row/column, or -1 for uncolored points.
    odometry: Pose
    rgb: np.ndarray | None = None  # Original raw RGB, before RTAB-Map rectification.
    camera: dict | None = None

    def __post_init__(self):
        if not 0 < self.node_id < 2**31:
            raise ValueError('keyframe node_id must be a positive int32')
        xyz = np.array(self.xyz, dtype=np.float32, copy=True)
        rgba = np.array(self.rgba, dtype=np.uint8, copy=True)
        pixels = np.array(self.pixels, dtype=np.int32, copy=True)
        if xyz.ndim != 2 or xyz.shape[1] != 3 or not np.isfinite(xyz).all():
            raise ValueError('keyframe points must be finite Nx3 laser coordinates')
        if not 0 < len(xyz) < 2**32 or rgba.shape != (len(xyz), 4) or pixels.shape != (len(xyz), 2):
            raise ValueError('keyframe colors and pixels must match laser rows')
        for name, value in [('xyz', xyz), ('rgba', rgba), ('pixels', pixels)]:
            value.flags.writeable = False
            object.__setattr__(self, name, value)
        if self.rgb is not None:
            rgb = np.array(self.rgb, dtype=np.uint8, copy=True)
            if self.camera is None or rgb.shape != (self.camera['height'], self.camera['width'], 3):
                raise ValueError('archived RGB must match its camera calibration')
            rgb.flags.writeable = False
            object.__setattr__(self, 'rgb', rgb)

    @property
    def ids(self):
        return np.uint64(self.node_id << 32) | np.arange(len(self.xyz), dtype=np.uint64)


class MapLedger:
    """Immutable local observations + replaceable optimized keyframe poses.

    Rebuilds are atomic to consumers: every array in a snapshot belongs to the
    same revision. Removed voxels are absent, rather than left as ghost points.
    """
    def __init__(self, *, resolution=.05, frame_id='map', session_id=None):
        if not np.isfinite(resolution) or resolution <= 0:
            raise ValueError('resolution must be positive and finite')
        self.resolution, self.frame_id = float(resolution), frame_id
        self.session_id = session_id or uuid.uuid4().hex
        self.keyframes, self.poses = {}, {}
        self.active = set()
        self.epoch_ns = None
        self.revision = 0
        self.dirty = False
        self.previous = None

    def add(self, frame, pose):
        if frame.node_id in self.keyframes:
            raise ValueError('stable keyframe IDs cannot be reused')
        if pose.wrd_frame != self.frame_id or pose.ego_frame != frame.odometry.ego_frame:
            raise ValueError('optimized keyframe pose has incompatible frames')
        if self.epoch_ns is not None and frame.stamp_ns < self.epoch_ns:
            raise ValueError('keyframe predates this map session')
        self.epoch_ns = frame.stamp_ns if self.epoch_ns is None else self.epoch_ns
        self.keyframes[frame.node_id] = frame
        self.poses[frame.node_id] = pose
        self.active.add(frame.node_id)
        self.dirty = True

    def update_graph(self, poses, *, complete=False):
        """Apply individual node poses, not one rigid map->odom correction.

        Partial working-memory graphs must not delete archived nodes. Only an
        explicitly complete graph may retire nodes absent from its ID set.
        """
        for node_id, pose in poses.items():
            if node_id not in self.keyframes:
                continue
            previous = self.poses[node_id]
            if (pose.wrd_frame, pose.ego_frame) != (previous.wrd_frame, previous.ego_frame):
                raise ValueError('optimized keyframe pose has incompatible frames')
            if not np.allclose(previous.matrix, pose.matrix, atol=1e-10, rtol=0):
                self.poses[node_id] = pose
                self.dirty = True
            if node_id not in self.active:
                self.active.add(node_id)
                self.dirty = True
        if complete:
            active = self.active.intersection(poses)
            self.dirty |= active != self.active
            self.active = active

    def snapshot(self, *, allocator=np.empty):
        if not self.dirty and self.previous is not None:
            return self.previous
        xyz, rgba, source_ids, stamps, pixels = [], [], [], [], []
        metadata = {}
        for node_id in sorted(self.active):
            frame, pose = self.keyframes[node_id], self.poses[node_id]
            xyz.append(pose(frame.xyz).astype(np.float32))
            rgba.append(frame.rgba)
            source_ids.append(frame.ids)
            stamps.append(np.full(len(frame.xyz), frame.stamp_ns, dtype=np.int64))
            pixels.append(frame.pixels)
            metadata[str(node_id)] = dict(stamp_ns=frame.stamp_ns, scan_stamp_ns=frame.scan_stamp_ns,
                point_count=len(frame.xyz), pose=pose.matrix.tolist(), odometry=frame.odometry.matrix.tolist(),
                camera=frame.camera)
        xyz = np.concatenate(xyz) if xyz else np.empty((0, 3), np.float32)
        rgba = np.concatenate(rgba) if rgba else np.empty((0, 4), np.uint8)
        ids = np.concatenate(source_ids) if source_ids else np.empty(0, np.uint64)
        stamps = np.concatenate(stamps) if stamps else np.empty(0, np.int64)
        pixels = np.concatenate(pixels) if pixels else np.empty((0, 2), np.int32)
        keys = point_keys_from_xyz(xyz, self.resolution)
        # Prefer an actually colored observation, then latest time and stable ID.
        # Keep real laser geometry rather than replacing it with a voxel center.
        order = np.lexsort((ids, stamps, rgba[:, 3] != 0, keys))
        selected = order[np.r_[keys[order][1:] != keys[order][:-1], True]] if len(order) else order
        relative_ms = (stamps[selected] - (self.epoch_ns or 0)) // 10**6
        if len(relative_ms) and (relative_ms.min() < 0 or relative_ms.max() > np.iinfo(np.int32).max):
            raise ValueError('session timestamps exceed graphmap int32 relative milliseconds')
        points = InfoPoints.from_arrays(xyz[selected], rgba[selected], relative_ms.astype(np.int32))
        arrays = dict(infopoints=points.data, point_keys=keys[selected], representative_ids=ids[selected],
                      source_ids=ids, source_keys=keys, source_stamps_ns=stamps, source_pixels=pixels)
        shared = {}
        for name, array in arrays.items():
            target = allocator(array.shape, array.dtype)
            target[...] = array
            target.flags.writeable = False
            shared[name] = target
        self.revision += 1
        snapshot = dict(schema=SCHEMA, session_id=self.session_id, revision=self.revision,
            frame_id=self.frame_id, resolution=self.resolution, epoch_ns=self.epoch_ns,
            keyframes=metadata, **shared)
        self.previous, self.dirty = snapshot, False
        return snapshot

    def save(self, directory):
        """Persist immutable local scans and latest graph; no pickle payloads."""
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        manifest = directory/'manifest.json'
        if manifest.exists() and json.loads(manifest.read_text())['session_id'] != self.session_id:
            raise ValueError('archive directory belongs to another map session')
        if not manifest.exists() and any(directory.glob('*.npz')):
            raise ValueError('archive has orphaned frames without a session manifest')
        frames = {}
        for node_id, frame in self.keyframes.items():
            path = directory/f'{node_id}.npz'
            if not path.exists():
                arrays = dict(xyz=frame.xyz, rgba=frame.rgba, pixels=frame.pixels)
                if frame.rgb is not None:
                    arrays['rgb'] = frame.rgb
                temporary = path.with_suffix('.npz.tmp')
                with temporary.open('wb') as file:
                    np.savez_compressed(file, **arrays)
                temporary.replace(path)
            frames[str(node_id)] = dict(stamp_ns=frame.stamp_ns, scan_stamp_ns=frame.scan_stamp_ns,
                odometry=frame.odometry.matrix.tolist(), pose=self.poses[node_id].matrix.tolist(),
                odom_frame=frame.odometry.wrd_frame, body_frame=frame.odometry.ego_frame, camera=frame.camera)
        metadata = dict(schema=SCHEMA, session_id=self.session_id, resolution=self.resolution,
            frame_id=self.frame_id, epoch_ns=self.epoch_ns, revision=self.revision,
            active=sorted(self.active), keyframes=frames)
        temporary = directory/'manifest.tmp'
        temporary.write_text(json.dumps(metadata, indent=2))
        temporary.replace(directory/'manifest.json')

    @classmethod
    def load(cls, directory):
        directory = Path(directory)
        metadata = json.loads((directory/'manifest.json').read_text())
        if metadata['schema'] != SCHEMA:
            raise ValueError('unsupported laser map archive')
        result = cls(resolution=metadata['resolution'], frame_id=metadata['frame_id'], session_id=metadata['session_id'])
        for key, record in metadata['keyframes'].items():
            with np.load(directory/f'{key}.npz', allow_pickle=False) as arrays:
                frame = LaserKeyframe(int(key), record['stamp_ns'], record['scan_stamp_ns'],
                    arrays['xyz'], arrays['rgba'], arrays['pixels'],
                    Pose.from_matrix(record['odometry'], wrd_frame=record['odom_frame'], ego_frame=record['body_frame']),
                    rgb=arrays['rgb'] if 'rgb' in arrays else None, camera=record.get('camera'))
            result.add(frame, Pose.from_matrix(record['pose'], wrd_frame=result.frame_id, ego_frame=record['body_frame']))
        result.active = set(metadata['active'])
        result.epoch_ns = metadata['epoch_ns']
        result.revision = metadata['revision']
        result.dirty = True
        return result


class GraphMap:
    """Application-owned features that survive revisions of a laser map.

    Feed ``(await mapper.map.get()).data`` to update(). Features live in IndexDBs
    keyed by stable observation ID. Voxel tables are derived for each revision.
    Directly changing a derived voxel table does not change the source features.
    """
    def __init__(self):
        self.snapshot = None
        self.feature_dbs = {}
        self._tables = {}

    def update(self, snapshot):
        if snapshot['schema'] != SCHEMA:
            raise ValueError('unsupported laser map snapshot')
        if self.snapshot is not None:
            if snapshot['session_id'] != self.snapshot['session_id']:
                raise ValueError('another map session requires a new GraphMap')
            if snapshot['revision'] < self.revision:
                raise ValueError('cannot apply an older map revision')
        self.snapshot = snapshot
        self.infopoints = InfoPoints(snapshot['infopoints'], metadata={
            'frame_id': snapshot['frame_id'], 'session_id': snapshot['session_id'],
            'revision': snapshot['revision'], 'epoch_ns': snapshot['epoch_ns'],
            'time_unit': 'milliseconds_since_epoch_ns'})
        self.point_keys = snapshot['point_keys']
        self.source_ids, self.source_keys = snapshot['source_ids'], snapshot['source_keys']
        self._source_order = np.argsort(self.source_keys, kind='stable')
        self._sorted_keys = self.source_keys[self._source_order]
        self._tables = {}
        return self

    @property
    def revision(self):
        return self.snapshot['revision']

    def voxel_for(self, source_id):
        row = np.searchsorted(self.source_ids, np.uint64(source_id))
        if row == len(self.source_ids) or self.source_ids[row] != source_id:
            raise KeyError(source_id)
        return int(self.source_keys[row])

    def sources_for(self, point_key):
        lo = np.searchsorted(self._sorted_keys, np.uint64(point_key), side='left')
        hi = np.searchsorted(self._sorted_keys, np.uint64(point_key), side='right')
        return self.source_ids[self._source_order[lo:hi]]

    def set_features(self, name, source_ids, values):
        if name in {'sources', 'representative'} or not name or '/' in name or '\\' in name or name in {'.', '..'}:
            raise ValueError('invalid or reserved feature table name')
        source_ids = [int(value) for value in source_ids]
        values = list(values)
        if len(source_ids) != len(values):
            raise ValueError('feature values must align with observation IDs')
        for source_id in source_ids:
            self.voxel_for(source_id)  # Validate before mutation.
        table = self.feature_dbs.setdefault(name, IndexDB(name))
        for source_id, value in zip(source_ids, values):
            table.put(source_id, value)
        self._tables.pop(name, None)

    def set_voxel_features(self, name, point_keys, values, *, revision):
        if revision != self.revision:
            raise ValueError('voxel features refer to a stale map revision')
        point_keys, values = list(point_keys), list(values)
        if len(point_keys) != len(values):
            raise ValueError('feature values must align with voxel keys')
        ids, expanded = [], []
        for key, value in zip(point_keys, values):
            sources = self.sources_for(key)
            if not len(sources):
                raise KeyError(key)
            ids.extend(map(int, sources))
            expanded.extend([value] * len(sources))
        self.set_features(name, ids, expanded)

    def index_db(self, name):
        """Voxel-key IndexDB. Feature conflicts retain ALL source-ID/value pairs."""
        if name in self._tables:
            return self._tables[name]
        table = IndexDB(name)
        if name == 'representative':
            for key, source_id in zip(self.point_keys, self.snapshot['representative_ids']):
                table.put(int(key), int(source_id))
        elif name == 'sources':
            for key in self.point_keys:
                table.put(int(key), self.sources_for(key))
        elif name in self.feature_dbs:
            grouped = {}
            for source_id in self.feature_dbs[name].point_to_internal:
                try:
                    key = self.voxel_for(source_id)
                except KeyError:
                    continue  # Inactive sources keep features, but leave no ghost voxel.
                grouped.setdefault(key, []).append(dict(source_id=source_id, value=self.feature_dbs[name].get(source_id)))
            for key, records in grouped.items():
                table.put(key, sorted(records, key=lambda record: record['source_id']))
        else:
            raise KeyError(name)
        self._tables[name] = table
        return table

    def environment(self):
        from graphmap.environment import Environment
        env = Environment(resolution=self.snapshot['resolution'])
        env.infopoints = self.infopoints
        env.index_dbs = {name: self.index_db(name) for name in ('sources', 'representative', *self.feature_dbs)}
        return env

    def save(self, directory):
        """Graphmap-compatible view plus stable feature tables and reindex data."""
        directory = Path(directory)
        self.environment().save(directory/'environment')
        feature_dir = directory/'source_features'
        feature_dir.mkdir(parents=True, exist_ok=True)
        for name, table in self.feature_dbs.items():
            table.save(feature_dir/f'{name}.pkl')
        arrays = {key: value for key, value in self.snapshot.items() if isinstance(value, np.ndarray)}
        metadata = {key: value for key, value in self.snapshot.items() if key not in arrays}
        metadata['feature_tables'] = sorted(self.feature_dbs)
        np.savez_compressed(directory/'snapshot.npz', **arrays)
        (directory/'snapshot.json').write_text(json.dumps(metadata, indent=2))

    @classmethod
    def load(cls, directory):
        """Load trusted graphmap archives (IndexDB payloads use pickle)."""
        directory = Path(directory)
        metadata = json.loads((directory/'snapshot.json').read_text())
        names = metadata.pop('feature_tables')
        with np.load(directory/'snapshot.npz', allow_pickle=False) as arrays:
            snapshot = {**metadata, **{key: arrays[key] for key in arrays}}
        result = cls().update(snapshot)
        result.feature_dbs = {name: IndexDB.load(directory/'source_features'/f'{name}.pkl') for name in names}
        return result
