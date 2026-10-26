"""Platform functions kept outside planning and the velocity controller."""
import asyncio
from contextlib import suppress
import json
import os
from pathlib import Path
import re
import time
import urllib.parse
import urllib.request

from .catalog import vendor_headers


class VendorSupport:
    def __init__(self, config, *, simulated=False):
        self.config, self.simulated = config, simulated
        self.mapping = dict(active=None, last_operation=None, error=None)
        self.lock = asyncio.Lock()

    def task(self, operation, body):
        if operation not in ('start', 'save', 'cancel'): raise ValueError('unsupported mapping operation')
        payload = {'command_name': 'mapping_' + operation}
        if operation == 'start': payload['exploration'] = False
        if operation == 'save':
            name = str(body.get('map_name', '')).strip()
            if not name or len(name) > 80: raise ValueError('map name must contain 1–80 characters')
            payload.update(map_name=name, floor_id=str(body.get('floor_id') or '1')[:40],
                           building_id=str(body.get('building_id') or 'zbz')[:40])
        if self.simulated: return {'ok': True, 'simulated': True, 'command': payload}
        config = self.config.get('catalog', {})
        base = config.get('vendor_url', '')
        if not re.fullmatch(r'http://127\.0\.0\.1:[0-9]+', base): raise ValueError('vendor URL must be loopback')
        data = urllib.parse.urlencode({'json': json.dumps(payload, ensure_ascii=False)}).encode()
        headers = dict(vendor_headers(config), **{'Content-Type': 'application/x-www-form-urlencoded'})
        request = urllib.request.Request(base + '/task', data=data, headers=headers)
        with urllib.request.urlopen(request, timeout=60 if operation == 'save' else 20) as response:
            result = json.loads(response.read(65536))
        if result.get('error_code') not in (0, '0'):
            raise ValueError(str(result.get('error_msg') or result.get('error') or 'vendor mapping rejected')[:300])
        return {'ok': True, 'vendor': result}

    async def map_operation(self, operation, body):
        if not self.config.get('allow_mapping', False): raise ValueError('mapping writes disabled in deployment')
        if self.lock.locked(): raise ValueError('platform action already in progress')
        async with self.lock:
            try:
                result = await asyncio.to_thread(self.task, operation, body)
                self.mapping.update(active=operation == 'start', last_operation=operation, error=None)
                return dict(result, mapping=self.mapping)
            except Exception as error:
                # A timeout is an unknown outcome, never an automatic retry.
                self.mapping.update(active=None, last_operation=operation, error=str(error))
                raise

    async def pedal(self, action):
        if not self.config.get('allow_pedal', False): raise ValueError('pedal writes disabled in deployment')
        values = {'stop': 0, 'extend': 1, 'retract': 2}
        if action not in values: raise ValueError('pedal action must be extend, retract or stop')
        if self.lock.locked(): raise ValueError('platform action already in progress')
        async with self.lock:
            if not self.simulated:
                interface = self.config.get('pedal_can', 'can1')
                if not re.fullmatch(r'can[0-9]+', interface): raise ValueError('invalid pedal CAN interface')
                frame = '604#2B014000%02X000000' % values[action]
                for _ in range(2):
                    process = await asyncio.create_subprocess_exec('/usr/bin/cansend', interface, frame,
                                                                   stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
                    try:
                        async with asyncio.timeout(3): _, error = await process.communicate()
                    except BaseException:
                        if process.returncode is None: process.kill()
                        await process.wait()
                        raise
                    if process.returncode: raise ValueError('pedal CAN send failed: ' + error.decode(errors='replace')[:160])
                    await asyncio.sleep(.02)
            return dict(ok=True, sent=True, action=action, position='unconfirmed', feedback=False,
                        simulated=self.simulated)


class VoiceSession:
    """An owned, on-demand Python 3.6 child; recognized intents never call OEM."""
    def __init__(self, config, data_dir, on_intent, lease_valid=lambda _: False):
        self.config, self.data_dir, self.on_intent = config, Path(data_dir), on_intent
        self.process = self.reader = None
        self.pending, self.sequence = {}, 0
        self.lock = asyncio.Lock()
        self.events = asyncio.Queue(maxsize=8)
        self.bound_lease = None
        self.lease_valid = lease_valid
        self.last_result = None
        self.cached = dict(running=False, navigation='RSim', available=bool(config.get('directory') and (Path(config['directory'])/'assistant.py').is_file()))

    async def ensure(self):
        async with self.lock:
            if self.process and self.process.returncode is None: return
            directory = self.config.get('directory')
            if not directory or not (Path(directory)/'assistant.py').is_file(): raise ValueError('platform audio modules unavailable')
            env = {k: v for k, v in os.environ.items() if k not in ('PYTHONPATH', 'PYTHONHOME', 'LD_LIBRARY_PATH')}
            # Only deployment config supplies ASR configuration. Never serialize
            # that environment into status, logs or HTTP responses.
            env.update({k: str(v) for k, v in self.config.get('environment', {}).items() if k.startswith('ONBOARD_')})
            self.data_dir.mkdir(parents=True, exist_ok=True)
            helper = Path(__file__).resolve().parents[3]/'compat'/'onboard_audio.py'
            self.process = await asyncio.create_subprocess_exec(self.config.get('python', '/usr/bin/python3'), '-u', str(helper),
                directory, str(self.data_dir/'voice-volume.json'), env=env,
                stdin=asyncio.subprocess.PIPE, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.DEVNULL,
                start_new_session=os.name != 'nt')
            self.reader = asyncio.create_task(self.read())

    async def read(self):
        try:
            while line := await self.process.stdout.readline():
                reply = json.loads(line)
                if reply.get('event') == 'intent':
                    if not self.events.full(): self.events.put_nowait(reply)
                elif (future := self.pending.get(reply.get('id'))) is not None and not future.done():
                    if reply.get('error'): future.set_exception(ValueError(reply['error']))
                    else: future.set_result(reply.get('result'))
        finally:
            for future in self.pending.values():
                if not future.done(): future.set_exception(RuntimeError('audio helper exited'))

    async def call(self, op, **data):
        await self.ensure()
        if op == 'context' and 'lease' in data: self.bound_lease = data['lease']
        if op == 'stop': self.bound_lease = None
        self.sequence += 1
        identifier = self.sequence
        future = asyncio.get_running_loop().create_future()
        self.pending[identifier] = future
        try:
            self.process.stdin.write((json.dumps(dict(id=identifier, op=op, data=data))+'\n').encode())
            await self.process.stdin.drain()
            async with asyncio.timeout(30 if op == 'once' else 10): return await future
        finally: self.pending.pop(identifier, None)

    async def status(self):
        if self.process and self.process.returncode is None:
            try: self.cached = await self.call('status')
            except (ValueError, RuntimeError, TimeoutError): self.cached = dict(running=False, error='audio helper unavailable')
        return dict(self.cached, last_dispatch=self.last_result)

    async def pump(self):
        while True:
            import inspect
            valid = self.lease_valid(self.bound_lease) if self.bound_lease else True
            if inspect.isawaitable(valid): valid = await valid
            if not valid:
                self.bound_lease = None
                try: await self.call('stop')
                except (ValueError, RuntimeError, TimeoutError, OSError): pass
            try: event = await asyncio.wait_for(self.events.get(), .3)
            except TimeoutError: continue
            try:
                if not 0 <= time.monotonic() - event['at'] <= 1.: raise ValueError('recognized intent expired')
                self.last_result = await self.on_intent(event['lease'], event['intent'])
            except (ValueError, TimeoutError, OSError) as error: self.last_result = dict(ok=False, error=str(error))

    async def close(self):
        process = self.process
        if process and process.returncode is None:
            process.stdin.close()
            try:
                async with asyncio.timeout(2): await process.wait()
            except TimeoutError:
                if os.name != 'nt':
                    import signal
                    with suppress(ProcessLookupError): os.killpg(process.pid, signal.SIGTERM)
                else: process.terminate()
                try:
                    async with asyncio.timeout(2): await process.wait()
                except TimeoutError:
                    process.kill()
                    await process.wait()
        if self.reader:
            self.reader.cancel()
            await asyncio.gather(self.reader, return_exceptions=True)
