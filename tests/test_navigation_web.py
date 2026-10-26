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


def test_exclusive_preemption_replay_expiry_and_zero_only(system):
    client, service = system
    lease = client.post('/api/claim').json()['lease']
    old = lease
    lease = client.post('/api/claim').json()['lease']
    assert client.post('/api/control', json=request(old, 1, 'arm')).status_code == 409
    expired = request(lease, 1, 'arm', deadline_ms=1)
    assert client.post('/api/control', json=expired).status_code == 409
    service.bridge.motion_enabled = False
    assert client.post('/api/control', json=request(lease, 1, 'arm')).status_code == 409
    service.bridge.motion_enabled = True
    assert client.post('/api/control', json=request(lease, 2, 'arm')).status_code == 200
    assert client.post('/api/control', json=request(lease, 2, 'goal', x=1, y=0, yaw=0)).status_code == 409
    assert client.post('/api/control', json=request(lease, 3, 'stop')).status_code == 200
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
    assert client.post('/api/control', json=request(lease, 3, 'stop')).status_code == 200
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


def test_takeover_revokes_old_stop_and_socket_disconnect(system):
    client, service = system
    old = client.post('/api/claim').json()['lease']
    with client.websocket_connect('/ws/control') as ws:
        ws.receive_json(); ws.send_json(request(old, 1, 'arm')); assert ws.receive_json()['ok']
        new = client.post('/api/claim').json()['lease']
        assert not service.armed and service.goal is None
        assert client.post('/api/control', json=request(new, 1, 'arm')).status_code == 200
        for sequence, op in enumerate(('manual', 'stop', 'pause', 'cancel', 'heartbeat'), 2):
            ws.send_json(request(old, sequence, op)); assert not ws.receive_json()['ok']
        assert client.post('/api/control', json={'op': 'stop'}).status_code == 409
        assert service.armed
    time.sleep(.05)
    assert service.armed  # closing the revoked socket cannot stop the new owner
    assert service.motion_owner == new


def test_shared_independent_sequences_and_only_active_disconnect_stops(system):
    client, service = system
    service.config['control_mode'] = 'shared'
    a = client.post('/api/claim').json()['lease']
    with client.websocket_connect('/ws/control') as wa:
        wa.receive_json(); wa.send_json(request(a, 1, 'arm')); assert wa.receive_json()['ok']
        b = client.post('/api/claim').json()['lease']
        assert service.armed  # joining shared mode must not interrupt motion
        assert client.post('/api/control', json=request(b, 1, 'manual', steering=.3)).status_code == 200
        assert service.motion_owner == b
        wa.send_json(request(a, 2, 'idle')); assert wa.receive_json()['ok']
        assert service.armed and service.axes == (0., .3)
        assert client.get('/api/status').json()['controller_count'] == 2
        assert client.post('/api/control', json=request(b, 1, 'heartbeat')).status_code == 409
    time.sleep(.05)
    assert service.armed and service.motion_owner == b
    assert client.post('/api/control', json=request(b, 2, 'manual', steering=-.3)).status_code == 200
    assert service.axes == (0., -.3)
    client.portal.call(service.release_controller, b)
    assert not service.armed and service.published == [0., 0.]


def test_shared_heartbeat_does_not_keep_another_driver_alive(system):
    client, service = system
    service.config['control_mode'] = 'shared'
    a = client.post('/api/claim').json()['lease']
    b = client.post('/api/claim').json()['lease']
    assert client.post('/api/control', json=request(a, 1, 'arm')).status_code == 200
    service.controllers[a]['deadline'] = 0
    assert client.post('/api/control', json=request(b, 1, 'heartbeat')).status_code == 200
    client.portal.call(service.tick)
    assert not service.armed and service.last_disarm['reason'] == 'controller_disconnected'


def test_ownership_setting_keeps_saver_and_revokes_other_pages(system):
    client, service = system
    service.config['control_mode'] = 'shared'
    a = client.post('/api/claim').json()['lease']
    b = client.post('/api/claim').json()['lease']
    d = client.get('/api/settings').json()
    d['values']['control_mode'] = 'exclusive'
    result = client.post('/api/settings', json=dict(lease=b, sequence=1,
        deadline_ms=time.monotonic()*1000+1500, revision=d['revision'], values=d['values']))
    assert result.status_code == 200, result.text
    assert result.json()['active']['control_mode'] == 'exclusive'
    assert service.controller_valid(b) and not service.controller_valid(a)
    assert not service.armed


def test_pending_goal_cannot_cross_exclusive_takeover(system):
    client, service = system
    old = client.post('/api/claim').json()['lease']
    ticket = client.portal.call(service.reserve_goal, request(old, 1, 'goal'))
    client.post('/api/claim')
    with pytest.raises(ValueError, match='superseded'):
        client.portal.call(service.commit_goal, ticket, dict(op='goal', x=1, y=0, yaw=0))
    assert service.goal is None


def test_manual_collision_holds_zero_without_revoking_arm_and_deadman_still_stops(system):
    import numpy as np
    from rsim.components.towing_navigation import Decision
    from rsim.core.commands import VelocityCommand
    client, service = system
    blocked = True
    def compute(*_):
        return (Decision(VelocityCommand(0., .1), 'blocked' if blocked else 'manual', not blocked,
                         np.empty((0,3)), np.zeros((1,3))), np.zeros(3), np.zeros(2), np.empty((0,2)))
    service.compute = compute
    lease = client.post('/api/claim').json()['lease']
    assert client.post('/api/control', json=request(lease,1,'arm')).status_code == 200
    assert client.post('/api/control', json=request(lease,2,'manual',steering=.3)).status_code == 200
    client.portal.call(service.tick)
    assert service.reason == 'blocked' and service.armed and service.published == [0.,0.]
    assert service.mode == 'manual'
    blocked = False
    assert client.post('/api/control', json=request(lease,3,'manual',steering=.3)).status_code == 200
    client.portal.call(service.tick)
    assert service.armed and service.published[1] > 0
    service.manual_deadline = 0
    client.portal.call(service.tick)
    assert not service.armed and service.published == [0.,0.]
    assert service.last_disarm['reason'] == 'manual_deadman'
