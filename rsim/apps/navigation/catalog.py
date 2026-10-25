"""Read the existing OEM map/point database without importing its control code."""
import ast
import hashlib
import io
import json
import math
from pathlib import Path
import re
import urllib.request

import numpy as np


def vendor_headers(config):
    token = ''
    path = config.get('vendor_token_config')
    if path:
        source = Path(path).read_text(encoding='utf8')
        match = re.search(r'TOKEN_WHITE_LIST\s*=\s*(\[[^\]]*\])', source, re.S)
        if match:
            values = ast.literal_eval(match.group(1))
            if values: token = str(values[0])
    return {'Authorization': token}


class MapCatalog:
    def __init__(self, config):
        self.config = config
        self.map_dir = Path(config['map_directory']) if config.get('map_directory') else None
        self.location_dir = Path(config.get('location_directory') or config.get('map_directory') or '.')

    @staticmethod
    def identifier(value):
        if not isinstance(value, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,100}', value):
            raise ValueError('invalid map identifier')
        return value

    def maps(self):
        if self.map_dir is None: return []
        result = []
        for path in sorted(self.map_dir.glob('*_map.json')):
            try:
                meta = json.loads(path.read_text(encoding='utf8'))
                identifier = self.identifier(meta.get('map_uuid') or path.name[:-9])
                origin = list(map(float, meta.get('origin', [0, 0, 0])))
                width, height = int(meta['width']), int(meta['height'])
                resolution = float(meta['resolution'])
                if len(origin) != 3 or not np.isfinite(origin).all() or width <= 0 or height <= 0 or width * height > 16_000_000 or not 0 < resolution < 10:
                    continue
                result.append(dict(map_uuid=identifier, name=str(meta.get('map_name') or identifier), width=width, height=height,
                                   resolution=resolution, origin=origin[:2], yaw=origin[2], frame='map',
                                   version='stored:' + hashlib.sha256(path.read_bytes()).hexdigest()[:20]))
            except (ValueError, KeyError, OSError, TypeError):
                continue
        return result

    def metadata(self, identifier):
        self.identifier(identifier)
        for item in self.maps():
            if item['map_uuid'] == identifier: return item
        raise ValueError('unknown stored map')

    def current_uuid(self):
        """The only vendor HTTP operation in the catalog is a read-only GET."""
        if self.map_dir is None: return None
        url = self.config.get('vendor_url', 'http://127.0.0.1:8088')
        if not re.fullmatch(r'http://127\.0\.0\.1:[0-9]+', url): raise ValueError('vendor URL must be loopback')
        request = urllib.request.Request(url + '/get_current_map', headers=vendor_headers(self.config))
        with urllib.request.urlopen(request, timeout=2) as response:
            payload = json.loads(response.read(65536))
        if payload.get('error_code') not in (None, 0, '0'): raise ValueError('vendor map query rejected')
        inner = payload.get('data', payload)
        return inner.get('map_uuid') or None

    def assert_active(self, identifier, occupancy):
        if identifier != self.current_uuid(): raise ValueError('selected map is not the active localization map')
        meta = self.metadata(identifier)
        if occupancy is None or (meta['width'], meta['height']) != (occupancy.width, occupancy.height):
            raise ValueError('stored map dimensions do not match the live map')
        if not np.allclose([meta['resolution'], *meta['origin'], meta['yaw']],
                           [occupancy.resolution, *occupancy.origin, occupancy.yaw], atol=1e-5):
            raise ValueError('stored map coordinates do not match the live map')

    def locations(self, identifier):
        meta = self.metadata(identifier)
        candidates = [self.location_dir / (identifier + '_location.json'), self.map_dir / (identifier + '_location.json')]
        path = next((p for p in candidates if p.is_file()), None)
        if path is None: return []
        raw = json.loads(path.read_text(encoding='utf8'))
        if isinstance(raw, dict): raw = raw.get('locations') or raw.get('data') or []
        if not isinstance(raw, list): raise ValueError('invalid location database')
        c, s = math.cos(meta['yaw']), math.sin(meta['yaw'])
        result = []
        for index, point in enumerate(raw):
            try:
                px, py = float(point['ix']), float(point['iy'])
                local = np.array([px, meta['height'] - py]) * meta['resolution']
                x, y = np.array([[c, -s], [s, c]]) @ local + meta['origin']
                # OEM image Y is down, but itheta itself is WORLD yaw in degrees.
                yaw = math.radians(float(point.get('itheta', point.get('yaw', 0))))
                if not np.isfinite([x, y, yaw]).all(): continue
                result.append(dict(id='oem:' + identifier + ':' + str(point.get('loc_uuid') or index),
                                   name=str(point.get('loc_name') or point.get('name') or '位置'),
                                   map_uuid=identifier, x=float(x), y=float(y), yaw=yaw,
                                   source='oem', editable=False))
            except (ValueError, KeyError, TypeError):
                continue
        return result

    def image(self, identifier):
        from PIL import Image
        meta = self.metadata(identifier)
        for suffix in ('_map.png', '.png', '_map.pgm', '.pgm'):
            path = self.map_dir / (identifier + suffix)
            if not path.is_file(): continue
            with Image.open(path) as picture:
                if picture.size != (meta['width'], meta['height']): raise ValueError('stored image dimensions differ from metadata')
                stream = io.BytesIO()
                picture.save(stream, format='PNG')
                return stream.getvalue()
        raise ValueError('stored map image unavailable')
