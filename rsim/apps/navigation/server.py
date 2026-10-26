"""Control API with optional token auth, a small WebSocket and separate visuals."""
from __future__ import annotations
import asyncio
from contextlib import asynccontextmanager
import hmac
import json
from pathlib import Path
import time
import hashlib
import uuid
import os
from copy import deepcopy
from typing import Literal
import urllib.request

from fastapi import FastAPI, HTTPException, Request, Response, WebSocket, WebSocketDisconnect, BackgroundTasks
from fastapi.responses import FileResponse, HTMLResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field, ConfigDict
from starlette.middleware.trustedhost import TrustedHostMiddleware

from ...runtime import Runtime


class Control(BaseModel):
    model_config = ConfigDict(extra='forbid', allow_inf_nan=False)
    op: Literal['heartbeat', 'idle', 'arm', 'manual', 'manual_collision', 'goal', 'pause', 'resume', 'stop', 'cancel', 'footprint', 'drive_mode', 'speeds']
    lease: str | None = Field(default=None, max_length=64)
    sequence: int | None = Field(default=None, strict=True, ge=1)
    deadline_ms: float | None = None
    throttle: float = Field(default=0, ge=-1, le=1)
    steering: float = Field(default=0, ge=-1, le=1)
    input_source: Literal['keys', 'joystick'] = 'keys'
    input_active: bool = Field(default=True, strict=True)
    collision_check: bool = Field(default=True, strict=True)
    x: float = Field(default=0, ge=-10000, le=10000)
    y: float = Field(default=0, ge=-10000, le=10000)
    yaw: float = Field(default=0, ge=-100, le=100)
    extended: bool | None = Field(default=None, strict=True)
    drive_mode: Literal['towing', 'differential'] | None = None
    map_uuid: str | None = Field(default=None, max_length=100)
    map_version: str | None = Field(default=None, max_length=100)
    navigation: float = Field(default=1., gt=0, le=1.6)
    manual: float = Field(default=1., gt=0, le=1.6)
    angular: float = Field(default=.8, gt=0, le=3)
    handles: bool = Field(default=True, strict=True)


def clock_ms(): return time.monotonic() * 1000


def create_app(service, token: str | None, *, own_runtime=True, data_dir=None, allowed_hosts=None, runtime_options=None, settings_store=None):
    if token is not None and len(token) < 24: raise ValueError('control token must contain at least 24 characters')
    data_dir = Path(data_dir or 'assets/navigation')
    from .settings import SettingsStore, apply_config
    settings = settings_store or SettingsStore(data_dir / 'settings.json', service.config)
    settings_lock = asyncio.Lock()
    static = Path(__file__).with_name('static')
    # Archive deployments can preserve old mtimes. Give each asset a content
    # version so WebView cannot reuse an older file after an HTML upgrade.
    index_html = (static / 'index.html').read_text(encoding='utf8')
    for asset in ('style.css', 'app.js', 'developer.js'):
        revision = hashlib.sha256((static / asset).read_bytes()).hexdigest()[:16]
        index_html = index_html.replace(f'/static/{asset}"', f'/static/{asset}?v={revision}"')
    from .catalog import MapCatalog
    catalog = MapCatalog(service.config.get('catalog', {}))
    from .auxiliary import VendorSupport, VoiceSession
    support = VendorSupport(service.config, simulated=service.simulated)

    @asynccontextmanager
    async def lifespan(app):
        pump = asyncio.create_task(voice.pump())
        try:
            if own_runtime:
                async with Runtime(service, options=runtime_options): yield
            else: yield
        finally:
            pump.cancel()
            await asyncio.gather(pump, return_exceptions=True)
            await voice.close()

    app = FastAPI(title='RSim Towing Navigation', lifespan=lifespan, docs_url=None, redoc_url=None, openapi_url=None)
    app.add_middleware(TrustedHostMiddleware, allowed_hosts=allowed_hosts or ['localhost', '127.0.0.1', '[::1]', 'testserver'])
    app.mount('/static', StaticFiles(directory=static), name='static')

    @app.exception_handler(ValueError)
    async def value_error(_request, error): return JSONResponse({'error': str(error)}, status_code=409)

    @app.exception_handler(TimeoutError)
    async def timeout_error(_request, _error): return JSONResponse({'error': 'operation timed out; outcome unconfirmed'}, status_code=504)

    def origin_ok(connection):
        origin = connection.headers.get('origin')
        if not origin: return True  # CLI clients use bearer auth; browsers send Origin on writes.
        from urllib.parse import urlsplit
        return urlsplit(origin).netloc == connection.headers.get('host') and urlsplit(origin).scheme in ('http', 'https')

    def authenticated(connection):
        if token is None: return True
        authorization = connection.headers.get('authorization', '')
        supplied = authorization[7:] if authorization.startswith('Bearer ') else connection.cookies.get('rsim_session', '')
        return bool(supplied and hmac.compare_digest(supplied.encode('utf8'), token.encode('utf8')))

    @app.middleware('http')
    async def security(request, call_next):
        public = request.url.path in ('/', '/api/session', '/api/health') or request.url.path.startswith('/static/')
        if request.method not in ('GET', 'HEAD') and not origin_ok(request):
            return JSONResponse({'error': 'origin rejected'}, status_code=403)
        if not public and not authenticated(request): return JSONResponse({'error': 'authentication required'}, status_code=401)
        if request.headers.get('content-length', '0').isdigit() and int(request.headers.get('content-length', 0)) > 8192:
            return JSONResponse({'error': 'request too large'}, status_code=413)
        response = await call_next(request)
        response.headers['X-Content-Type-Options'] = 'nosniff'
        response.headers['Referrer-Policy'] = 'no-referrer'
        response.headers['Content-Security-Policy'] = "default-src 'self'; img-src 'self' blob:; connect-src 'self'; script-src 'self'; style-src 'self'; frame-ancestors 'none'"
        if request.url.path.startswith('/api/'): response.headers['Cache-Control'] = 'no-store'
        return response

    @app.get('/')
    async def index(): return HTMLResponse(index_html, headers={'Cache-Control': 'no-store'})

    @app.get('/legacy')
    async def legacy_page():
        return HTMLResponse((static / 'legacy.html').read_text(encoding='utf8'), headers={'Cache-Control': 'no-store'})

    @app.post('/api/legacy/{operation}')
    async def legacy_service(operation: str, request: Request):
        if not service.config.get('legacy_service', False): raise HTTPException(404, 'legacy service not configured')
        if operation not in ('start', 'stop'): raise HTTPException(400, 'invalid operation')
        body = await request.json()
        await service.command(dict(body, op='cancel'))
        script = Path(__file__).resolve().parents[3] / 'scripts' / 'legacy_hih_service.py'
        process = await asyncio.create_subprocess_exec('/usr/bin/python3', str(script), operation,
            stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
        try:
            async with asyncio.timeout(30): output, error = await process.communicate()
        except BaseException:
            if process.returncode is None: process.kill()
            await process.wait()
            raise
        if process.returncode: raise HTTPException(503, output.decode(errors='replace')[:200])
        return json.loads(output)

    @app.get('/api/health')
    async def health():
        return {'ok': service._failure is None, 'zero_only': not service.bridge.motion_enabled,
                'simulated': service.simulated, 'auth_required': token is not None}

    @app.post('/api/session')
    async def login(request: Request):
        if token is None: return {'ok': True, 'clock_ms': clock_ms()}
        body = await request.json()
        supplied = str(body.get('token', '')) if isinstance(body, dict) else ''
        if not hmac.compare_digest(supplied.encode('utf8'), token.encode('utf8')): raise HTTPException(401, 'invalid token')
        result = JSONResponse({'ok': True, 'clock_ms': clock_ms()})
        result.set_cookie('rsim_session', token, httponly=True, samesite='strict',
                          secure=request.url.scheme == 'https', max_age=8 * 3600)
        return result

    @app.get('/api/clock')
    async def clock(): return {'clock_ms': clock_ms()}

    @app.get('/api/status')
    async def status(compact: bool = False):
        return dict(service.status(compact=compact), capabilities=dict(mapping=service.config.get('allow_mapping', False),
                    pedal=service.config.get('allow_pedal', False), audio=service.config.get('allow_audio', False),
                    localization=service.simulated or service.config.get('ros', {}).get('allow_localization', False)))

    @app.post('/api/claim')
    async def claim():
        try: return {'lease': await service.claim(), 'clock_ms': clock_ms()}
        except ValueError as error: raise HTTPException(409, str(error))

    async def execute(command):
        if command.op not in ('stop', 'pause', 'cancel'):
            now = clock_ms()
            maximum = 2000 if command.op in ('goal', 'footprint', 'drive_mode', 'speeds') else 500
            if command.deadline_ms is None or not now < command.deadline_ms <= now + maximum:
                raise ValueError('control deadline invalid for ' + command.op)
        data = command.model_dump(exclude_none=True)
        if command.op == 'goal' and command.map_uuid:
            ticket = await service.reserve_goal(data)
            await asyncio.to_thread(catalog.assert_active, command.map_uuid, service.map)
            await service.commit_goal(ticket, data)
        else: await service.command(data)
        if command.op in ('speeds', 'drive_mode', 'footprint'):
            async with settings_lock:
                values = deepcopy(settings.values)
                if command.op == 'speeds': values['speed_limits'] = {k: data[k] for k in ('navigation', 'manual', 'angular', 'handles')}
                elif command.op == 'drive_mode': values['drive_mode'] = data['drive_mode']
                else: values['footprint']['rear'] = .81 if data['extended'] else .27
                settings.save(values, settings.revision)
                key = {'speeds': 'speed_limits', 'drive_mode': 'drive_mode', 'footprint': 'footprint'}[command.op]
                service.config[key] = deepcopy(settings.active[key])
        return {'ok': True, 'op': command.op, 'sequence': command.sequence, 'clock_ms': clock_ms(), 'mode': service.mode}

    @app.post('/api/control')
    async def control(command: Control):
        try: return await execute(command)
        except (ValueError, TimeoutError) as error: raise HTTPException(409, str(error))

    @app.websocket('/ws/control')
    async def socket(ws: WebSocket):
        if not authenticated(ws) or not origin_ok(ws):
            await ws.close(code=1008)
            return
        await ws.accept()
        owner = None
        pending = set()
        window, count = time.monotonic(), 0
        await ws.send_json({'clock_ms': clock_ms(), 'hello': 'rsim-control-v1'})
        async def dispatch(command):
            try:
                reply = await execute(command)
            except (ValueError, TimeoutError, OSError) as error:
                reply = {'ok': False, 'op': command.op, 'error': str(error), 'sequence': command.sequence, 'clock_ms': clock_ms()}
            try: await ws.send_json(reply)
            except (RuntimeError, WebSocketDisconnect): pass
        try:
            while True:
                raw = await asyncio.wait_for(ws.receive_text(), 3)
                if len(raw) > 4096: break
                now = time.monotonic()
                if now - window > 1.: window, count = now, 0
                count += 1
                if count > 40: raise ValueError('control rate exceeds 40 Hz')
                command = Control.model_validate_json(raw)
                if owner is None: owner = command.lease
                elif command.lease != owner: raise ValueError('socket controller cannot change')
                if command.op == 'goal' and command.map_uuid:
                    # Vendor map-identity reads may be slow. Continue receiving
                    # manual/stop/heartbeat packets; a late goal must still pass
                    # the controller's original deadline and sequence checks.
                    if len(pending) >= 2: raise ValueError('too many pending goals')
                    task = asyncio.create_task(dispatch(command))
                    pending.add(task)
                    task.add_done_callback(pending.discard)
                else:
                    await dispatch(command)
        except (WebSocketDisconnect, TimeoutError, ValueError):
            pass
        finally:
            for task in pending: task.cancel()
            await asyncio.gather(*pending, return_exceptions=True)
            if owner is not None:
                await service.release_controller(owner)
            try: await ws.close()
            except RuntimeError: pass

    @app.get('/api/map/meta')
    async def map_meta():
        if service.map is None: raise HTTPException(503, 'map unavailable')
        return service.map.metadata()

    @app.get('/api/maps')
    async def maps():
        try:
            items, current = await asyncio.gather(asyncio.to_thread(catalog.maps), asyncio.to_thread(catalog.current_uuid))
            return {'maps': items, 'current_map_uuid': current}
        except Exception as error: raise HTTPException(503, str(error))

    @app.get('/api/maps/{identifier}/image')
    async def stored_image(identifier: str):
        try: return Response(await asyncio.to_thread(catalog.image, identifier), media_type='image/png')
        except (ValueError, OSError) as error: raise HTTPException(404, str(error))

    @app.get('/api/map.png')
    async def map_image(request: Request):
        current = service.map
        if current is None: raise HTTPException(503, 'map unavailable')
        tag = '"' + current.version + '"'
        if request.headers.get('if-none-match') == tag: return Response(status_code=304)
        return Response(current.png or await asyncio.to_thread(current.image), media_type='image/png', headers={'ETag': tag})

    @app.get('/api/visual')
    async def visual():
        try: service.observed_points = await asyncio.to_thread(service.scan_points)
        except ValueError: service.observed_points = service.observed_points[:0]
        # Bounded display sampling only. Collision checking retains all points.
        def bounded(points, maximum):
            return points[::max(1, math_ceil(len(points) / maximum))].round(3).tolist()
        return {'path': bounded(service.path, 250), 'trajectory': bounded(service.trajectory, 120),
                'waypoints': service.status().get('planning', {}).get('waypoints', []),
                'scan': bounded(service.observed_points, 600), 'pose': service.pose_value,
                'footprint': service.footprint.corners(service.pose_value).tolist() if service.pose_value else []}

    @app.post('/api/localization')
    async def localization(command: Control):
        try:
            if command.deadline_ms is None or not clock_ms() < command.deadline_ms < clock_ms() + 2000:
                raise ValueError('localization deadline expired')
            ticket = await service.reserve_goal(command.model_dump(exclude_none=True))
            if command.map_uuid: await asyncio.to_thread(catalog.assert_active, command.map_uuid, service.map)
            if service.map is None or (command.map_version and command.map_version != service.map.version):
                raise ValueError('map changed before localization')
            result = await service.relocalize_reserved(ticket, command.x, command.y, command.yaw)
            if not result.get('ok'): raise HTTPException(409, result.get('error'))
            return result
        except (ValueError, TimeoutError) as error: raise HTTPException(409, str(error))

    async def auxiliary_authorize(body):
        if not isinstance(body, dict): raise ValueError('expected an object')
        if not clock_ms() < float(body.get('deadline_ms', 0)) <= clock_ms() + 2000: raise ValueError('auxiliary request expired')
        await service.authorize_request(body.get('lease'), body.get('sequence'))

    def lifecycle_state():
        path = os.environ.get('RSIM_LIFECYCLE_STATE')
        try:
            state = json.loads(Path(path).read_text()) if path else {}
            return state if state.get('child_pid') == os.getpid() and time.time() - state.get('updated_at', 0) < 10 else {}
        except (OSError, ValueError): return {}

    @app.get('/api/settings')
    async def get_settings():
        return dict(settings.view(), restart_available=bool(lifecycle_state()), lifecycle=lifecycle_state())

    @app.post('/api/settings')
    async def save_settings(request: Request):
        body = await request.json()
        await auxiliary_authorize(body)
        async with settings_lock:
            values = settings.check(body.get('values'), body.get('revision'))
            previous = deepcopy(settings.active)
            active = settings.live(values)
            await service.apply_settings(active, lease=body['lease'])
            try: settings.save(values, body['revision'])
            except (OSError, ValueError):
                await service.apply_settings(previous)
                raise
            apply_config(service.config, active)
            return dict(ok=True, **settings.view())

    @app.post('/api/service/restart')
    async def restart_service(request: Request, background: BackgroundTasks):
        body = await request.json()
        await auxiliary_authorize(body)
        if not lifecycle_state(): raise ValueError('service was not started by the managed launcher')
        if service.status().get('maintenance'): raise ValueError('finish the auxiliary operation before restarting')
        await service.stop('service_restart', cancel=True)
        async def request_restart():
            await asyncio.sleep(.4)
            target = Path(os.environ['RSIM_RESTART_REQUEST'])
            temporary = target.with_suffix('.next')
            temporary.write_text(json.dumps(dict(child_pid=os.getpid(), requested_at=time.time())))
            temporary.replace(target)
        background.add_task(request_restart)
        return dict(ok=True, restarting=True)

    @app.get('/api/mapping')
    @app.get('/api/mapping/status')
    async def mapping_status():
        return dict(support.mapping, enabled=service.config.get('allow_mapping', False),
                    telemetry=service.status().get('telemetry', {}).get('mapping'))

    @app.get('/api/mapping/preview.png')
    async def mapping_preview(request: Request): return await map_image(request)

    @app.post('/api/mapping/{operation}')
    async def mapping(operation: Literal['start', 'save', 'cancel'], request: Request):
        body = await request.json()
        if not service.config.get('allow_mapping', False): raise ValueError('mapping writes disabled in deployment')
        await auxiliary_authorize(body)
        if support.lock.locked(): raise ValueError('platform action already in progress')
        await service.set_maintenance('mapping_unknown')
        result = await support.map_operation(operation, body)
        await service.set_maintenance('mapping' if operation == 'start' else None)
        return result

    @app.post('/api/pushrod')
    async def pushrod(request: Request):
        body = await request.json()
        await auxiliary_authorize(body)
        return await pedal(body.get('action'))

    async def pedal(action):
        if not service.config.get('allow_pedal', False): raise ValueError('pedal writes disabled in deployment')
        if action not in ('extend', 'retract', 'stop'): raise ValueError('invalid pedal action')
        if support.lock.locked(): raise ValueError('platform action already in progress')
        await service.set_maintenance('pedal')
        await service.pedal_action(action)
        try: return await support.pedal(action)
        finally: await service.set_maintenance(None)

    @app.get('/api/battery')
    async def battery(): return service.status().get('telemetry', {}).get('battery', {'fresh': False, 'data': None})

    @app.get('/api/handles')
    async def handles(): return service.status().get('telemetry', {}).get('handles', {'fresh': False, 'data': None})

    @app.get('/api/speed')
    async def speed(): return {'measured': service.status().get('measured'), 'source': 'ROS odometry'}

    @app.get('/api/speeds')
    async def speeds(): return service.status().get('speed_limits', {})

    @app.post('/api/speeds')
    async def set_speeds(request: Request):
        body = await request.json()
        return await execute(Control.model_validate(dict(body, op='speeds')))

    def local_waypoints():
        path = data_dir / 'waypoints.json'
        entries = json.loads(path.read_text(encoding='utf8')) if path.exists() else []
        for item in entries:
            item.setdefault('id', hashlib.sha256(json.dumps(item, sort_keys=True).encode()).hexdigest()[:20])
            item.update(source='rsim', editable=True)
        return entries

    def write_waypoints(entries):
        data_dir.mkdir(parents=True, exist_ok=True)
        temporary = data_dir / 'waypoints.next'
        temporary.write_text(json.dumps(entries, ensure_ascii=False), encoding='utf8')
        temporary.replace(data_dir / 'waypoints.json')

    @app.get('/api/waypoints')
    async def waypoints(map_uuid: str | None = None):
        entries = local_waypoints()
        if map_uuid:
            try: existing = await asyncio.to_thread(catalog.locations, map_uuid)
            except (ValueError, OSError) as error: raise HTTPException(404, str(error))
            return existing + [p for p in entries if p.get('map_uuid') == map_uuid]
        return [p for p in entries if service.map is not None and p.get('map_version') == service.map.version]

    @app.post('/api/waypoints')
    async def save_waypoint(request: Request):
        body = await request.json()
        values = [float(body[k]) for k in ('x', 'y', 'yaw')]
        import math
        if not all(math.isfinite(x) for x in values) or service.map is None: raise HTTPException(422, 'invalid waypoint')
        if not service.map.known_free(values[:2]): raise HTTPException(422, 'waypoint is outside known free space')
        if body.get('map_version') and body['map_version'] != service.map.version: raise HTTPException(409, 'map changed')
        if body.get('map_uuid'):
            try: await asyncio.to_thread(catalog.assert_active, body['map_uuid'], service.map)
            except ValueError as error: raise HTTPException(409, str(error))
        existing = local_waypoints()
        if len(existing) >= 200: raise HTTPException(409, 'waypoint limit reached')
        entry = dict(id=uuid.uuid4().hex, name=str(body.get('name', 'Waypoint'))[:80], x=values[0], y=values[1],
                     yaw=values[2], map_version=service.map.version, map_uuid=body.get('map_uuid'))
        existing.append(entry)
        write_waypoints(existing)
        return entry

    @app.post('/api/waypoints/{identifier}')
    async def edit_waypoint(identifier: str, request: Request):
        body = await request.json()
        entries = local_waypoints()
        entry = next((p for p in entries if p['id'] == identifier), None)
        if entry is None: raise HTTPException(404, 'RSim waypoint not found; OEM entries are read-only')
        if body.get('delete') is True: entries.remove(entry)
        else:
            name = str(body.get('name', '')).strip()[:80]
            if not name: raise HTTPException(422, 'name is required')
            entry['name'] = name
        write_waypoints(entries)
        return {'ok': True}

    async def location_list(map_uuid=None):
        identifier = map_uuid or await asyncio.to_thread(catalog.current_uuid)
        points = await waypoints(identifier)
        return [dict(p, loc_name=p['name'], loc_uuid=p['id']) for p in points]

    @app.get('/api/state')
    async def legacy_state(map_uuid: str | None = None):
        return dict(await status(), locations=await location_list(map_uuid), maps=await maps(),
                    nav_running=service.mode == 'navigation', mapping=await mapping_status())

    @app.get('/api/dispatch/locations')
    async def dispatch_locations(map_uuid: str | None = None): return {'ok': True, 'locations': await location_list(map_uuid)}

    @app.get('/api/dispatch/status')
    async def dispatch_status(): return await status()

    @app.get('/api/dispatch/map/current')
    async def dispatch_map(): return await maps()

    @app.get('/api/dispatch/map/image')
    async def dispatch_image(request: Request, map_uuid: str | None = None):
        return await stored_image(map_uuid) if map_uuid else await map_image(request)

    async def location_goal(body):
        if body.get('loc_uuid') or body.get('location_id'):
            points = await location_list(body.get('map_uuid'))
            identifier = body.get('loc_uuid') or body.get('location_id')
            point = next((p for p in points if p['id'] == identifier or p['id'].rsplit(':', 1)[-1] == identifier), None)
            if point is None: raise ValueError('waypoint is not in the selected map')
            return {k: point[k] for k in ('x', 'y', 'yaw', 'map_uuid') if point.get(k) is not None}
        return {k: body[k] for k in ('x', 'y', 'yaw', 'map_uuid', 'map_version') if k in body}

    @app.post('/api/goto')
    @app.post('/api/dispatch/goto')
    async def goto(request: Request):
        body = await request.json()
        target = await location_goal(body)
        if not all(k in target for k in ('x', 'y', 'yaw')): raise ValueError('goal coordinates or waypoint required')
        command = {k: body[k] for k in ('lease', 'sequence', 'deadline_ms') if k in body}
        return await execute(Control(op='goal', **command, **target))

    async def navigation_action(request: Request):
        operation = request.url.path.rsplit('/', 1)[-1]
        body = await request.json()
        command = {k: body[k] for k in ('lease', 'sequence', 'deadline_ms') if k in body}
        return await execute(Control(op=operation, **command))

    for operation in ('pause', 'resume', 'cancel'):
        app.add_api_route('/api/' + operation, navigation_action, methods=['POST'])
        app.add_api_route('/api/dispatch/' + operation, navigation_action, methods=['POST'])

    async def voice_intent(lease, parsed):
        if not parsed.get('ok'): raise ValueError('unrecognized voice intent')
        intent = parsed.get('intent')
        if intent in ('pedal_extend', 'pedal_retract'):
            # No implicit claim/arming, and no expired voice session may move
            # the actuator. The stop command itself is always harmless.
            if not await service.validate_controller(lease): raise ValueError('voice controller disconnected')
            return await pedal(intent.removeprefix('pedal_'))
        op = {'goto_location': 'goal', 'cancel_nav': 'cancel', 'pause_nav': 'pause', 'resume_nav': 'resume'}.get(intent)
        if op is None: raise ValueError('unsupported voice intent')
        target = {}
        if op == 'goal':
            point = parsed.get('location') or {}
            target = await location_goal({'loc_uuid': point.get('loc_uuid') or point.get('id'), 'map_uuid': point.get('map_uuid')})
            if target.get('map_uuid'): await asyncio.to_thread(catalog.assert_active, target['map_uuid'], service.map)
            target['map_version'] = service.map.version if service.map else None
        await service.intent(lease, dict(op=op, deadline_ms=clock_ms()+400, **target))
        return {'ok': True, 'action': op, 'controller': 'RSim'}

    voice = VoiceSession(service.config.get('voice', {}), data_dir, voice_intent,
                         service.validate_controller)
    app.state.voice = voice
    app.state.support = support

    @app.get('/api/assistant/status')
    async def assistant_status(): return await voice.status()

    @app.post('/api/assistant/{operation}')
    async def assistant(operation: Literal['text', 'once', 'start', 'stop'], request: Request):
        body = await request.json()
        if operation == 'stop': return await voice.call('stop') if voice.process else {'ok': True}
        if operation != 'text' and not service.config.get('allow_audio', False): raise ValueError('microphone disabled in deployment')
        await auxiliary_authorize(body)
        locations = await location_list()
        await voice.call('context', locations=locations, lease=body['lease'], audio=service.config.get('allow_audio', False))
        return await voice.call(operation, text=str(body.get('text', ''))[:400])

    @app.post('/api/voice/{operation}')
    async def audio_operation(operation: Literal['volume', 'announce'], request: Request):
        body = await request.json()
        if not service.config.get('allow_audio', False): raise ValueError('audio writes disabled in deployment')
        await auxiliary_authorize(body)
        await voice.call('context', audio=True)
        if operation == 'volume': return await voice.call('volume', volume_pct=int(body.get('volume_pct', 50)))
        return await voice.call('announce', key=str(body.get('key', 'depart_generic')))

    return app


def math_ceil(value):
    import math
    return math.ceil(value)
