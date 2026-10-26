"""Auxiliary controls must not create an alternate motion route."""
import asyncio
import json
import sys
import time
from pathlib import Path

import pytest
pytest.importorskip('pyrobo')
pytest.importorskip('fastapi')
from fastapi.testclient import TestClient
from rsim.apps.navigation.auxiliary import VendorSupport, VoiceSession
from rsim.apps.navigation.server import create_app
from rsim.apps.navigation.service import NavigationService
from rsim.apps.navigation.simulation import SimulationBridge


@pytest.fixture
def rig(tmp_path):
    config = dict(optional_scans=[], allow_mapping=False, allow_pedal=False, allow_audio=False)
    controller = NavigationService(SimulationBridge(), config, simulated=True)
    app = create_app(controller, None, data_dir=tmp_path)
    with TestClient(app) as client:
        for _ in range(60):
            if controller.map is not None: break
            time.sleep(.03)
        lease = client.post('/api/claim').json()['lease']
        sequence = 0
        def command(**data):
            nonlocal sequence
            sequence += 1
            return dict(lease=lease, sequence=sequence, deadline_ms=time.monotonic()*1000+1500, **data)
        yield client, app, controller, command


def test_disabled_physical_writes_have_no_side_effects(rig, monkeypatch):
    client, app, controller, command = rig
    def forbidden(*args, **kwargs): raise AssertionError('hardware called in disabled deployment')
    monkeypatch.setattr(app.state.support, 'task', forbidden)
    monkeypatch.setattr(asyncio, 'create_subprocess_exec', forbidden)
    for path, body in [('/api/mapping/start', {}), ('/api/pushrod', {'action': 'extend'}),
                       ('/api/voice/volume', {'volume_pct': 50}), ('/api/assistant/once', {})]:
        result = client.post(path, json=command(**body))
        assert result.status_code == 409, result.text
    assert controller.maintenance is None
    assert controller.published == [0., 0.]


def test_mapping_and_pedal_simulation_are_separate_from_drive_mode(rig):
    client, app, controller, command = rig
    controller.config.update(allow_mapping=True, allow_pedal=True)
    assert client.post('/api/control', json=command(op='drive_mode', drive_mode='differential')).status_code == 200
    assert client.post('/api/control', json=command(op='footprint', extended=False)).status_code == 200
    assert client.post('/api/pushrod', json=command(action='retract')).json()['position'] == 'unconfirmed'
    assert controller.footprint.rear == .81
    assert controller.drive_mode == 'differential'
    result = client.post('/api/mapping/start', json=command(map_name='Test'))
    assert result.json()['command'] == {'command_name': 'mapping_start', 'exploration': False}
    assert controller.maintenance == 'mapping'
    assert client.post('/api/control', json=command(op='goal', x=1, y=0, yaw=0)).status_code == 409
    body = command(op='manual', throttle=.3, steering=0)
    body['deadline_ms'] = time.monotonic()*1000+400
    assert client.post('/api/control', json=body).status_code == 200
    assert controller.mode == 'manual'
    result = client.post('/api/mapping/save', json=command(map_name='Test'))
    assert result.status_code == 200, result.text
    assert controller.maintenance is None
    assert controller.published == [0., 0.]


def test_voice_intent_uses_existing_lease_and_keeps_manual_priority(rig):
    client, app, controller, command = rig
    saved = client.post('/api/waypoints', json={'name': '大厅', 'x': 1, 'y': 0, 'yaw': 0}).json()
    body = command(op='manual', throttle=.3, steering=0)
    body['deadline_ms'] = time.monotonic()*1000+400
    assert client.post('/api/control', json=body).status_code == 200
    lease = body['lease']
    sequence = controller.controllers[lease]['sequence']
    intent = dict(ok=True, intent='goto_location', location={'loc_uuid': saved['id']})
    result = client.portal.call(app.state.voice.on_intent, lease, intent)
    assert result['controller'] == 'RSim'
    assert controller.mode == 'manual'
    assert controller.goal.tolist() == [1., 0., 0.]
    assert controller.controllers[lease]['sequence'] == sequence
    assert controller.armed is False
    controller.controllers[lease]['deadline'] = 0.
    with pytest.raises(ValueError, match='disconnected'):
        client.portal.call(app.state.voice.on_intent, lease, dict(ok=True, intent='resume_nav'))


def test_reserved_goal_survives_heartbeat_but_not_stop(rig):
    client, app, controller, command = rig
    request = command(op='goal', x=1, y=0, yaw=0)
    ticket = client.portal.call(controller.reserve_goal, request)
    heartbeat = command(op='heartbeat'); heartbeat['deadline_ms'] = time.monotonic()*1000+400
    assert client.post('/api/control', json=heartbeat).status_code == 200
    client.portal.call(controller.commit_goal, ticket, request)
    assert controller.mode == 'navigation'
    request = command(op='goal', x=2, y=0, yaw=0)
    ticket = client.portal.call(controller.reserve_goal, request)
    client.post('/api/control', json=command(op='stop'))
    with pytest.raises(ValueError, match='stopped'): client.portal.call(controller.commit_goal, ticket, request)


def test_delayed_localization_cannot_run_after_stop(rig):
    client, app, controller, command = rig
    request = command(op='goal', x=1, y=0, yaw=0)
    ticket = client.portal.call(controller.reserve_goal, request)
    client.post('/api/control', json=command(op='stop'))
    with pytest.raises(ValueError, match='stopped'):
        client.portal.call(controller.relocalize_reserved, ticket, 1, 0, 0)
    assert controller.bridge.position.tolist() == [0., 0., 0.]


def test_vendor_mapping_is_allowlisted_form_api(monkeypatch):
    import urllib.parse
    import urllib.request
    calls = []
    class Reply:
        def __enter__(self): return self
        def __exit__(self, *args): pass
        def read(self, _): return b'{"error_code":0,"data":{}}'
    def open_(request, **kwargs):
        calls.append(request)
        return Reply()
    monkeypatch.setattr(urllib.request, 'urlopen', open_)
    vendor = VendorSupport({'catalog': {'vendor_url': 'http://127.0.0.1:8088'}})
    assert vendor.task('save', {'map_name': '新地图'})['ok']
    assert calls[0].full_url == 'http://127.0.0.1:8088/task'
    body = json.loads(urllib.parse.parse_qs(calls[0].data.decode())['json'][0])
    assert body['command_name'] == 'mapping_save'
    with pytest.raises(ValueError): vendor.task('goto', {})
    assert len(calls) == 1


def test_audio_child_cannot_use_legacy_http_and_closes(tmp_path):
    (tmp_path/'voice.py').write_text("VOL_FILE='absent'\ndef volume_profile(): return {'volume_pct': 50}\n")
    (tmp_path/'assistant.py').write_text('''
def _play_prompt(key): raise AssertionError('unexpected audio')
def _http_json(*args, **kwargs): raise AssertionError('legacy HTTP called')
def status(): return {'running': False}
def stop_loop(): pass
def handle_text_command(text):
    assert _http_json('POST', '/api/goto')['error'] == 'legacy_navigation_disabled'
    return dispatch_intent({'ok':True, 'intent':'pause_nav'})
''')
    async def run():
        seen = []
        async def accept(lease, parsed): seen.append((lease, parsed)); return {'ok': True}
        session = VoiceSession({'directory': str(tmp_path), 'python': sys.executable}, tmp_path/'state', accept,
                               lease_valid=lambda lease: lease == 'unit-lease')
        pump = asyncio.create_task(session.pump())
        try:
            await session.call('context', lease='unit-lease', audio=False, locations=[])
            result = await session.call('text', text='pause')
            assert result['action'] == 'queued_for_rsim'
            async with asyncio.timeout(3):
                while not seen: await asyncio.sleep(.01)
            assert seen[0] == ('unit-lease', {'ok': True, 'intent': 'pause_nav'})
        finally:
            pump.cancel(); await asyncio.gather(pump, return_exceptions=True)
            await session.close()
        assert session.process.returncode is not None
    asyncio.run(run())
