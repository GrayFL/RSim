import numpy as np
import pytest

from rsim.core import Component
from rsim.components.scan_filter import AngularScanMask


def test_occluded_beams_are_unknown_without_compressing_scan_time():
    source = Component().signal('scan', clock='ros:system')
    component = AngularScanMask(source, angle_masks=[[-.1, .1]])
    scan = dict(header={'frame_id':'laser'}, angle_min=-.2, angle_increment=.1,
                angle_max=.2, time_increment=.001, scan_time=.005,
                range_min=.1, range_max=10., ranges=np.array([1.,2.,3.,4.,5.]),
                intensities=np.array([10.,20.,30.,40.,50.]))
    result = component.convert(scan)
    assert np.isnan(result['ranges'][2])
    assert result['ranges'][0] == 1 and result['ranges'][-1] == 5
    assert result['intensities'][2] == 0
    assert result['time_increment'] == .001 and result['angle_increment'] == .1
    assert result['header'] == scan['header']
    assert len(result['ranges']) == 5
    np.testing.assert_array_equal(scan['ranges'], [1,2,3,4,5])
    assert result['scan_time'] == scan['scan_time']


def test_ros1_sectors_match_all_four_chassis_posts_and_preserve_forward():
    component = AngularScanMask(Component().signal('scan'), angle_masks=[
        [-2.34,-1.95],[-1.1,-.75],[.75,1.1],[1.95,2.34]])
    angles = np.arange(-180,181)
    value = dict(angle_min=-np.pi, angle_increment=np.pi/180,
                 ranges=np.ones(len(angles)), intensities=[])
    result = component.convert(value)['ranges']
    for degree in [-123,-53,53,123]:
        assert np.isnan(result[degree+180])
    assert result[180] == 1 and result[270] == 1 and result[0] == 1
    assert np.isfinite(value['ranges']).all()


@pytest.mark.parametrize('masks', [[], [[1,-1]], [[0,4]], [[0,float('nan')]]])
def test_bad_scan_masks_are_rejected(masks):
    with pytest.raises(ValueError, match='angle_masks'):
        AngularScanMask(Component().signal('scan'), angle_masks=masks)


def test_native_recipe_keeps_raw_source_and_filtered_ros_topic_distinct(tmp_path):
    from rsim.drivers import NativeChassis
    from graphmap.pose import Pose
    robot = NativeChassis(stm32={'start_driver':False},imu={'topic':'/imu'},
        imu_mount=Pose(wrd_frame='base_footprint',ego_frame='imu'), directory=tmp_path,
        estimator='scan_imu', scan_mount=Pose(wrd_frame='base_footprint',ego_frame='laser'),
        scan={'topic':'/raw', 'filter':{'topic':'/filtered','angle_masks':[[-.2,.2]]}})
    assert robot.odometry.scan.scan_topic == '/filtered'
    assert robot.scan_filter in robot.odometry.scan.dependencies
    assert robot.scan is robot.scan_filter.scan
    assert robot.scan_filter.source.producer.source.producer.topic == '/raw'
