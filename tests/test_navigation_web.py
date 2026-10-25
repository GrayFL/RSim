"""API/authentication, control ownership and disconnect safety against simulation."""
import json
import hashlib
import re
import time
from pathlib import Path
import pytest

pytest.importorskip('pyrobo')
pytest.importorskip('fastapi')
from fastapi.testclient import TestClient
from rsim.apps.navigation.server import create_app
from rsim.apps.navigation.service import NavigationService
from rsim.apps.navigation.simulation import SimulationBridge

TOKEN = 'unit-test-token-never-used-on-a-robot'


@pytest.fixture(params=[True, False], ids=['token', 'no-token'])
def system(tmp_path, request):
    config = json.loads((Path(__file__).parents[1] / 'examples' / 'ros1_towing.json').read_text())
    config['optional_scans'] = []
    bridge = SimulationBridge()
    service = NavigationService(bridge, config, simulated=True)
    app = create_app(service, TOKEN if request.param else None, data_dir=tmp_path)
    with TestClient(app) as client:
        if request.param: client.headers['Authorization'] = 'Bearer ' + TOKEN
        for _ in range(60):
            if client.get('/api/status').json()['map']: break
            time.sleep(.05)
        assert service.map is not None
        yield client, service


def request(lease, sequence, op, **extra):
    return dict(op=op, lease=lease, sequence=sequence, deadline_ms=extra.pop('deadline_ms', time.monotonic() * 1000 + 300), **extra)


def test_auth_origin_map_and_validation(system):
    client, service = system
    auth_required = 'Authorization' in client.headers
    assert client.get('/api/health').status_code == 200
    assert client.get('/api/health').json()['auth_required'] is auth_required
    assert client.get('/api/status', headers={'Authorization': ''}).status_code == (401 if auth_required else 200)
    assert client.post('/api/control', json={'op': 'stop'}, headers={'Origin': 'https://evil.invalid'}).status_code == 403
    assert client.post('/api/control', json={'op': 'manual', 'throttle': -1.1}).status_code == 422
    login = client.post('/api/session', json={'token': '无效'})
    assert login.status_code == (401 if auth_required else 200)
    assert 'set-cookie' not in login.headers
    png = client.get('/api/map.png')
    assert png.content.startswith(b'\x89PNG')
    assert client.get('/api/map.png', headers={'If-None-Match': png.headers['etag']}).status_code == 304
    assert len(client.get('/api/status?compact=true').content) < 3000
    page = client.get('/', headers={'If-Modified-Since': 'Fri, 31 Dec 9999 23:59:59 GMT'})
    assert page.status_code == 200
    assert page.headers['cache-control'] == 'no-store'
    assert "frame-ancestors 'none'" in page.headers['content-security-policy']
    assets = re.findall(r'(?:src|href)="(/static/[^\"]+)"', page.text)
    assert len(assets) == 3
    for url in assets:
        response = client.get(url)
        assert response.status_code == 200
        assert url.endswith('?v=' + hashlib.sha256(response.content).hexdigest()[:16])


def test_single_owner_replay_expiry_and_zero_only(system):
    client, service = system
    lease = client.post('/api/claim').json()['lease']
    assert client.post('/api/claim').status_code == 409
    expired = request(lease, 1, 'arm', deadline_ms=1)
    assert client.post('/api/control', json=expired).status_code == 409
    service.bridge.motion_enabled = False
    assert client.post('/api/control', json=request(lease, 1, 'arm')).status_code == 409
    service.bridge.motion_enabled = True
    assert client.post('/api/control', json=request(lease, 2, 'arm')).status_code == 200
    assert client.post('/api/control', json=request(lease, 2, 'goal', x=1, y=0, yaw=0)).status_code == 409
    assert client.post('/api/control', json={'op': 'stop'}).status_code == 200
    assert not service.armed


def test_manual_disconnect_and_deadman(system):
    client, service = system
    lease = client.post('/api/claim').json()['lease']
    with client.websocket_connect('/ws/control') as ws:
        assert 'hello' in ws.receive_json()
        ws.send_json(request(lease, 1, 'arm'))
        assert ws.receive_json()['ok']
        for sequence in range(2, 7):
            ws.send_json(request(lease, sequence, 'manual', throttle=.4, steering=.5))
            assert ws.receive_json()['ok']
            time.sleep(.1)
        assert service.published[0] > 0  # Only the ROS-free simulator may move.
    time.sleep(.1)
    assert service.published == [0., 0.]
    assert not service.armed


def test_late_planner_result_cannot_restart_after_stop(system):
    client, service = system
    lease = client.post('/api/claim').json()['lease']
    real_compute = service.compute
    def slow(*args):
        time.sleep(.3)
        return real_compute(*args)
    service.compute = slow
    assert client.post('/api/control', json=request(lease, 1, 'arm')).status_code == 200
    assert client.post('/api/control', json=request(lease, 2, 'manual', throttle=.3, steering=0)).status_code == 200
    time.sleep(.15)
    start = time.monotonic()
    assert client.post('/api/control', json={'op': 'stop'}).status_code == 200
    assert time.monotonic() - start < .2
    time.sleep(.35)
    assert service.published == [0., 0.]
    assert not service.armed


def test_slow_map_catalog_does_not_block_manual_packets(system, monkeypatch):
    from rsim.apps.navigation.catalog import MapCatalog
    client, service = system
    monkeypatch.setattr(MapCatalog, 'assert_active', lambda *args: time.sleep(.6))
    lease = client.post('/api/claim').json()['lease']
    with client.websocket_connect('/ws/control') as ws:
        ws.receive_json()
        ws.send_json(request(lease, 1, 'arm'))
        assert ws.receive_json()['ok']
        ws.send_json(request(lease, 2, 'goal', map_uuid='test-map', x=1, y=0, yaw=0,
                             deadline_ms=time.monotonic()*1000+1200))
        start = time.monotonic()
        ws.send_json(request(lease, 3, 'manual', throttle=.2, steering=0))
        reply = ws.receive_json()
        assert reply['ok'] and reply['sequence'] == 3
        assert time.monotonic() - start < .3
        assert service.mode == 'manual'
