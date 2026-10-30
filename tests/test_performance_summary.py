"""A publication step can contain several native physics substeps."""
from copy import deepcopy

from tools.summarize_matrix import validate_navigation


def report():
    return dict(case='mujoco-realtime',factory=dict(hz=50.,physics=dict(timestep=.002)),
        navigation=[dict(arrived=True,collided=False,samples=[
            dict(session='s',episode=0,step_index=1,stamp_ns=20_000_000,speed=0.,turn=0.),
            dict(session='s',episode=0,step_index=2,stamp_ns=40_000_000,speed=0.,turn=0.)])],
        runtime=[dict(source_final=dict(stamp_ns=42_000_000,physical_substeps=21))])


def test_native_substeps_and_publication_steps_have_distinct_units():
    # Pausing may stop between normal 20ms publications at a real 2ms boundary.
    data = report()
    assert validate_navigation(data) == []
    wrong = deepcopy(data)
    wrong['runtime'][0]['source_final']['physical_substeps'] = 20
    assert 'physical dt' in validate_navigation(wrong)[0]


def test_publication_gap_must_match_source_time_and_generation():
    data = report()
    data['navigation'][0]['samples'][1]['step_index'] = 3
    assert 'publication interval' in validate_navigation(data)[0]
    data = report()
    data['navigation'][0]['samples'][1]['episode'] = 1
    assert validate_navigation(data)
