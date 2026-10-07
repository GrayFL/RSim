import numpy as np
import pytest

pytest.importorskip('graphmap.pose')
from graphmap.pose import Pose
from graphmap.index_db import point_keys_from_xyz

from rsim.components.mapping import LaserKeyframe, MapLedger, GraphMap
from rsim.components.projection import PinholeCamera, PoseHistory, colorize_laser


def keyframe(node, xyz, stamp=10**18, colors=None):
    xyz = np.asarray(xyz)
    return LaserKeyframe(node, stamp, stamp-10**7, xyz,
        np.full((len(xyz), 4), 255, np.uint8) if colors is None else colors,
        np.zeros((len(xyz), 2), np.int32), Pose(wrd_frame='odom', ego_frame='base'))


def mapped_pose(x=0, yaw=0):
    return Pose(x=x, yaw=yaw, wrd_frame='map', ego_frame='base')


def test_loop_correction_moves_voxels_preserves_ids_features_and_merge_split():
    ledger = MapLedger(resolution=.1)
    ledger.add(keyframe(1, [[1, 0, 0]]), mapped_pose())
    ledger.add(keyframe(2, [[1, 0, 0]], stamp=10**18+10**9), mapped_pose(x=1))
    graph = GraphMap().update(ledger.snapshot())
    ids = graph.source_ids.copy()
    original_keys = graph.source_keys.copy()
    graph.set_features('semantic', ids, ['wall', 'door'])
    revision = graph.revision
    # Different per-node corrections: the observations merge into one voxel.
    ledger.update_graph({1: mapped_pose(x=2), 2: mapped_pose(x=2)})
    graph.update(ledger.snapshot())
    assert len(graph.infopoints) == 1
    np.testing.assert_array_equal(graph.source_ids, ids)
    key = graph.point_keys[0]
    np.testing.assert_array_equal(graph.sources_for(key), ids)
    assert all(graph.voxel_for(source) == key for source in ids)
    assert graph.index_db('semantic').get(key) == [
        dict(source_id=int(ids[0]), value='wall'), dict(source_id=int(ids[1]), value='door')]
    assert not set(map(int, original_keys)) & set(map(int, graph.index_db('sources').indices))
    with pytest.raises(ValueError, match='stale'):
        graph.set_voxel_features('new', [key], [42], revision=revision)
    # Later optimization separates them again, each carrying its own feature.
    ledger.update_graph({2: mapped_pose(x=4, yaw=90)})
    graph.update(ledger.snapshot())
    assert len(graph.infopoints) == 2
    assert graph.index_db('semantic').get(graph.voxel_for(ids[0]))[0]['value'] == 'wall'
    assert graph.index_db('semantic').get(graph.voxel_for(ids[1]))[0]['value'] == 'door'
    np.testing.assert_allclose(ledger.keyframes[1].xyz, [[1, 0, 0]])
    np.testing.assert_array_equal(graph.point_keys, point_keys_from_xyz(graph.infopoints.xyz, .1))


def test_graph_retirement_is_explicit_and_features_do_not_leave_ghost_voxels():
    ledger = MapLedger()
    ledger.add(keyframe(1, [[1, 0, 0]]), mapped_pose())
    ledger.add(keyframe(2, [[2, 0, 0]]), mapped_pose())
    graph = GraphMap().update(ledger.snapshot())
    ids = graph.source_ids.copy()
    graph.set_features('tag', ids, [1, 2])
    ledger.update_graph({1: mapped_pose()})
    graph.update(ledger.snapshot())
    assert len(graph.infopoints) == 2  # Working-memory subset is not a deletion.
    ledger.update_graph({1: mapped_pose()}, complete=True)
    graph.update(ledger.snapshot())
    assert len(graph.infopoints) == 1 and graph.index_db('tag').num_links == 1
    with pytest.raises(KeyError):
        graph.voxel_for(ids[1])
    ledger.update_graph({2: mapped_pose(x=5)})
    graph.update(ledger.snapshot())
    assert graph.index_db('tag').get(graph.voxel_for(ids[1]))[0]['value'] == 2


def test_map_archives_and_graphmap_environment_round_trip(tmp_path):
    from dataclasses import replace
    from graphmap.environment import Environment
    ledger = MapLedger(resolution=.2)
    raw_rgb = np.arange(12, dtype=np.uint8).reshape(2, 2, 3)
    frame = replace(keyframe(2000000000, [[-1, 2, 3], [4, 5, 6]]), rgb=raw_rgb,
                    camera=dict(width=2, height=2, frame_id='optical'))
    ledger.add(frame, mapped_pose())
    graph = GraphMap().update(ledger.snapshot())
    ids = graph.source_ids.copy()
    graph.set_features('embedding', ids, [np.array([1., 2.]), np.array([3., 4.])])
    graph.save(tmp_path/'view')
    ledger.save(tmp_path/'ledger')
    restored = MapLedger.load(tmp_path/'ledger')
    np.testing.assert_array_equal(restored.keyframes[2000000000].rgb, raw_rgb)
    assert restored.keyframes[2000000000].camera['frame_id'] == 'optical'
    graph2 = GraphMap.load(tmp_path/'view')
    restored.update_graph({2000000000: mapped_pose(x=1)})
    graph2.update(restored.snapshot())
    np.testing.assert_array_equal(graph2.source_ids, ids)  # No float32 ID packing.
    np.testing.assert_array_equal(graph2.feature_dbs['embedding'].get(int(ids[0])), [1., 2.])
    env = Environment.load(tmp_path/'view/environment')
    assert len(env.infopoints) == 2
    assert env.index_db('sources').num_links == 2
    np.testing.assert_array_equal(env.point_keys(), graph.point_keys)
    assert not graph.infopoints.index.any()


def test_voxel_feature_assignment_tracks_all_current_sources_and_session_identity():
    ledger = MapLedger(resolution=.2)
    ledger.add(keyframe(1, [[1, 0, 0], [1.01, 0, 0]]), mapped_pose())
    graph = GraphMap().update(ledger.snapshot())
    graph.set_voxel_features('label', graph.point_keys, ['shelf'], revision=graph.revision)
    assert graph.feature_dbs['label'].num_links == 2
    with pytest.raises(ValueError, match='session'):
        graph.update({**graph.snapshot, 'session_id': 'another'})
    with pytest.raises(ValueError, match='reused'):
        ledger.add(keyframe(1, [[1, 1, 1]]), mapped_pose())


def test_empty_snapshot_and_uncolored_geometry_remain_valid(tmp_path):
    from rsim.transport.shared import SharedStore
    ledger = MapLedger()
    store = SharedStore(tmp_path/'shared')
    graph = GraphMap().update(ledger.snapshot(allocator=store.allocate))
    assert graph.infopoints.shape == (0, 6)
    assert isinstance(graph.snapshot['infopoints'], np.memmap)
    rgba = np.array([[80, 90, 100, 255], [128, 128, 128, 0]], np.uint8)
    ledger.add(keyframe(1, [[1, 0, 0], [1.01, 0, 0]], colors=rgba), mapped_pose())
    graph.update(ledger.snapshot())
    np.testing.assert_array_equal(graph.infopoints.color, [[80, 90, 100, 255]])
    assert len(graph.sources_for(graph.point_keys[0])) == 2


def test_rgb_projection_motion_compensation_occlusion_and_outside_geometry():
    camera = PinholeCamera(20, 20, np.array([[10., 0, 10], [0, 10, 10], [0, 0, 1]]), np.zeros(5))
    rgb = np.zeros((20, 20, 3), np.uint8)
    rgb[10, 5] = [200, 20, 10]
    # Optical convention is used just for this fixture; transforms are explicit.
    scan = np.array([[0, 0, 2], [-1, 0, 4], [100, 0, 2], [0, 0, -1.]])
    body, rgba, pixels = colorize_laser(scan,
        scan_pose=Pose(wrd_frame='odom', ego_frame='imu'),
        image_pose=Pose(x=1, wrd_frame='odom', ego_frame='base'),
        body_to_optical=Pose(wrd_frame='base', ego_frame='optical'), camera=camera, rgb=rgb)
    np.testing.assert_allclose(body, scan - [1, 0, 0])
    np.testing.assert_array_equal(rgba[0], [200, 20, 10, 255])
    np.testing.assert_array_equal(pixels[0], [10, 5])
    assert not rgba[1:, 3].any()  # Farther same-pixel point occluded; others outside.
    assert len(body) == len(scan)  # Color selection never removes laser geometry.


def test_pose_interpolation_is_bracketed_and_bounded():
    history = PoseHistory(max_gap_s=.15)
    pose0 = Pose(yaw=170, wrd_frame='odom', ego_frame='base')
    pose1 = Pose(x=2, yaw=-170, wrd_frame='odom', ego_frame='base')
    history.add(1000000000, pose0)
    history.add(1100000000, pose1)
    middle = history.at(1050000000)
    np.testing.assert_allclose(middle.position, [1, 0, 0])
    assert abs(abs(middle.euler[2]) - 180) < 1e-7
    assert history.at(1200000000) is None
    assert not history.add(1000000000, pose1)
    history.add(1500000000, pose1)
    assert history.at(1250000000) is None


def test_raw_rgb_projection_uses_lens_distortion():
    pytest.importorskip('cv2')
    camera = PinholeCamera(300, 200, [[100, 0, 100], [0, 100, 80], [0, 0, 1]], [.4, 0, 0, 0, 0])
    rgb = np.zeros((200, 300, 3), np.uint8)
    # x/z=.5; radial factor 1+.4*.5**2=1.1 -> u=155, not undistorted 150.
    rgb[80, 155] = [10, 20, 30]
    _, rgba, pixels = colorize_laser(np.array([[1., 0, 2.]]),
        scan_pose=Pose(wrd_frame='odom', ego_frame='imu'),
        image_pose=Pose(wrd_frame='odom', ego_frame='base'),
        body_to_optical=Pose(wrd_frame='base', ego_frame='optical'), camera=camera, rgb=rgb)
    np.testing.assert_array_equal(pixels, [[80, 155]])
    np.testing.assert_array_equal(rgba, [[10, 20, 30, 255]])
