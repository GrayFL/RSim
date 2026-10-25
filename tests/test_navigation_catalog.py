import json
import math
from pathlib import Path

import numpy as np
import pytest
from PIL import Image

from rsim.apps.navigation.catalog import MapCatalog


def test_existing_maps_points_pixel_coordinates_and_no_mutation(tmp_path):
    meta = dict(map_uuid='map-one', map_name='一层', width=80, height=60, resolution=.05, origin=[-2, -1, math.pi/2])
    (tmp_path/'map-one_map.json').write_text(json.dumps(meta), encoding='utf8')
    (tmp_path/'map-one_location.json').write_text(json.dumps({'locations': [
        dict(loc_uuid='desk', loc_name='前台', ix=10, iy=20, itheta=90),
        dict(loc_uuid='bad', ix='bad', iy=0)
    ]}), encoding='utf8')
    Image.fromarray(np.full((60, 80), 254, np.uint8)).save(tmp_path/'map-one_map.pgm')
    before = {p.name: p.read_bytes() for p in tmp_path.iterdir()}
    catalog = MapCatalog({'map_directory': str(tmp_path)})
    assert catalog.maps()[0]['name'] == '一层'
    point, = catalog.locations('map-one')
    assert [point['x'], point['y'], point['yaw']] == pytest.approx([-4, -.5, math.pi/2])
    assert point['editable'] is False
    assert catalog.image('map-one').startswith(b'\x89PNG')
    assert before == {p.name: p.read_bytes() for p in tmp_path.iterdir()}
    with pytest.raises(ValueError): catalog.metadata('../map-one')
    with pytest.raises(ValueError): catalog.locations('missing')


def test_map_selection_cannot_navigate_on_a_different_map(tmp_path):
    from types import SimpleNamespace
    meta = dict(map_uuid='map-one', width=80, height=60, resolution=.05, origin=[-2, -1, 0])
    (tmp_path/'map-one_map.json').write_text(json.dumps(meta))
    catalog = MapCatalog({'map_directory': str(tmp_path)})
    occupancy = SimpleNamespace(width=80, height=60, resolution=.05, origin=[-2, -1], yaw=0)
    catalog.current_uuid = lambda: 'map-two'
    with pytest.raises(ValueError, match='active localization'): catalog.assert_active('map-one', occupancy)
    catalog.current_uuid = lambda: 'map-one'
    catalog.assert_active('map-one', occupancy)
    occupancy.yaw = .1
    with pytest.raises(ValueError, match='coordinates'): catalog.assert_active('map-one', occupancy)
