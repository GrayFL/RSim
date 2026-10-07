from types import SimpleNamespace

import numpy as np
import pytest

from rsim.adapters.ros2.scan_odometry import ScanOdometry, scan_interval


def scan(stamp=1., frame='laser', increment=.01):
    return SimpleNamespace(header=SimpleNamespace(frame_id=frame,
        stamp=SimpleNamespace(sec=int(stamp), nanosec=round((stamp-int(stamp))*1e9))),
        ranges=[1.]*6, time_increment=increment)


def test_scan_interval_preserves_last_beam_and_rejects_unknown_timing():
    assert scan_interval(scan()) == (1_000_000_000, 1_050_000_000)
    for increment in [0., -.01, float('nan'), .2]:
        with pytest.raises(ValueError, match='timing'):
            scan_interval(scan(increment=increment))


def component(tmp_path):
    from rsim.adapters.ros2 import RosContext
    from graphmap.pose import Pose
    return ScanOdometry(RosContext(), prefix='/rsim/test', directory=tmp_path,
        mount=Pose(position=[.1,.2,.3], rotation=[0,0,10], degrees=True,
                   wrd_frame='base_footprint', ego_frame='laser'))


def test_scan_waits_for_full_wheel_coverage_and_rejects_internal_gaps(tmp_path):
    c=component(tmp_path); sent=[];c.scan=SimpleNamespace(publish=sent.append)
    c.wheel_times.extend([990_000_000,1_020_000_000])
    c.receive_scan(scan())
    assert not sent and len(c.pending)==1
    c.wheel_times.append(1_060_000_000);c.flush()
    assert len(sent)==1 and not c.pending
    c.wheel_times.extend([1_100_000_000,1_300_000_000])
    c.receive_scan(scan(1.15))
    assert len(sent)==1 and c.counts['uncovered']==1
    c.receive_scan(scan(1.4,frame='wrong'))
    assert c.counts['invalid']==1


def test_private_tf_and_native_frame_contract(tmp_path):
    c=component(tmp_path)
    assert c.driver.remappings['/tf']=='/rsim/test/scan2d/tf'
    assert c.driver.remappings['/tf_static']=='/rsim/test/scan2d/tf_static'
    import yaml
    params=yaml.safe_load((tmp_path/'scan2d-parameters.yaml').read_text())['/**']['ros__parameters']
    assert params['frame_id']=='base_footprint'
    assert params['odom_frame_id']==c.odom_frame
    assert params['deskewing'] and not params['publish_tf']
    with pytest.raises(ValueError,match='owns'):
        ScanOdometry(c.ros,prefix=c.prefix,mount=c.mount,directory=tmp_path,
                     parameters={'frame_id':'laser'})


def test_invalid_output_quaternion_is_not_healthy(tmp_path):
    c=component(tmp_path)
    msg=SimpleNamespace(header=SimpleNamespace(frame_id=c.odom_frame,
        stamp=SimpleNamespace(sec=1,nanosec=0)),child_frame_id='base_footprint',
        pose=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=0,y=0,z=0),
            orientation=SimpleNamespace(x=0,y=0,z=0,w=np.nan))))
    with pytest.raises(RuntimeError,match='lost tracking'):
        c.receive_odometry(msg)
    assert c.last_output is None


@pytest.mark.parametrize('fuse', [False, True])
def test_mapping_recipe_keeps_scan_frontend_independent_from_super_lio(tmp_path, fuse):
    from rsim.core import Component
    from rsim.adapters.ros2 import RosContext
    from rsim.drivers.mapping import mapping_graph
    class Inputs(Component):
        def __init__(self):
            self.ros=RosContext();self.prefix='/rsim/test'
            super().__init__(self.ros)
        def diagnostics(self):
            return {}
    mounts={f'body_{name}':dict(wrd_frame='base_footprint',ego_frame=name)
            for name in ['imu','lidar','camera','scan']}
    graph=mapping_graph(connection={},lidar_ip='',mounts=mounts,database=str(tmp_path/'map.db'),
        name='test',timing={},topics={},camera_parameters={},lidar_parameters={},
        lio_parameters={},rtabmap_parameters={},cloud_filter={},input_source=Inputs(),scan2d_parameters={},
        fusion_parameters={} if fuse else None)
    planar=graph.scan_odometry
    assert planar in (graph.fusion.children if fuse else graph.children)
    lio=next(child for child in graph.children if getattr(child,'package',None)=='super_lio')
    assert not any(k.startswith('lio.planar.') for k in lio.parameters)
    if fuse:
        assert graph.fusion in graph.children
        assert graph.mapping.fusion is graph.fusion
        assert graph.frames['lio'] != graph.frames['odom']
    else:
        assert graph.fusion is None
