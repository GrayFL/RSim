#!/usr/bin/env python3
"""Python 3.6 host for the installed audio modules, never the old controller.

JSON lines over inherited pipes are the entire control API. Recognition emits
intents; the parent RSim service decides whether they may affect its task.
There is deliberately no HTTP navigation client in this process.
"""
import importlib.util
import json
import os
import sys
import threading
import time


def main():
    directory, volume_file = sys.argv[1:3]
    output_lock = threading.Lock()
    context = {'locations': [], 'lease': None, 'audio': False}
    work_lock = threading.Lock()
    output = sys.stdout
    sys.stdout = sys.stderr  # Installed modules cannot corrupt the protocol.

    def emit(value):
        with output_lock:
            output.write(json.dumps(value, ensure_ascii=True) + '\n')
            output.flush()

    def load(name):
        spec = importlib.util.spec_from_file_location(name, os.path.join(directory, name + '.py'))
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
        return module

    voice = load('voice')
    original_volume = voice.VOL_FILE
    voice.VOL_FILE = volume_file
    if not os.path.exists(volume_file):
        try:
            with open(original_volume) as handle: initial = json.load(handle)
            with open(volume_file, 'w') as handle: json.dump(initial, handle)
        except (OSError, ValueError): pass
    assistant = load('assistant')
    assistant.NAV_BASE = 'disabled://rsim-intent-pipe'
    assistant._http_json = lambda *args, **kwargs: {'ok': False, 'error': 'legacy_navigation_disabled'}
    assistant.fetch_locations = lambda: list(context['locations'])
    original_prompt = assistant._play_prompt
    assistant._play_prompt = lambda key: original_prompt(key) if context['audio'] else False

    def intent(parsed):
        emit({'event': 'intent', 'intent': parsed, 'lease': context['lease'], 'at': time.monotonic()})
        return {'ok': True, 'action': 'queued_for_rsim'}

    assistant.dispatch_intent = intent
    loop = None

    def operation(request):
        nonlocal loop
        try:
            op = request['op']
            data = request.get('data') or {}
            if op == 'status':
                result = dict(assistant.status(), volume=voice.volume_profile(), navigation='RSim')
            elif op == 'context':
                context.update(data)
                result = {'ok': True}
            elif op == 'stop':
                assistant.stop_loop()
                context['lease'] = None
                result = {'ok': True}
            elif op == 'start':
                if loop is not None and loop.is_alive(): raise ValueError('recognizer already running')
                if not context['audio']: raise ValueError('microphone disabled')
                loop = threading.Thread(target=assistant.run_loop, daemon=True)
                loop.start()
                result = {'ok': True}
            elif op in ('text', 'once', 'volume', 'announce'):
                if not work_lock.acquire(False): raise ValueError('audio operation busy')
                try:
                    if op == 'text': result = assistant.handle_text_command(str(data['text'])[:400])
                    elif op == 'once':
                        if not context['audio']: raise ValueError('microphone disabled')
                        if loop is not None and loop.is_alive(): raise ValueError('continuous recognition is running')
                        result = assistant.listen_once(seconds=min(5., max(1., float(data.get('seconds', 3)))))
                    elif op == 'volume':
                        if not context['audio']: raise ValueError('audio writes disabled')
                        result = voice.set_volume(max(0, min(100, int(data['volume_pct']))))
                    else:
                        if not context['audio']: raise ValueError('audio writes disabled')
                        key = data['key']
                        if key not in ('depart_generic', 'arrive_generic', 'turn_left', 'turn_right', 'obstacle'):
                            raise ValueError('unknown prompt')
                        result = {'ok': bool(voice.play_key(key))}
                finally: work_lock.release()
            else: raise ValueError('unsupported audio operation')
            emit({'id': request['id'], 'result': result if isinstance(result, dict) else {'ok': True, 'value': result}})
        except Exception as error:
            emit({'id': request.get('id'), 'error': str(error)[:300]})

    try:
        for line in sys.stdin:
            if len(line) > 262144: break
            request = json.loads(line)
            if request['op'] in ('once', 'text', 'announce', 'volume'):
                threading.Thread(target=operation, args=(request,), daemon=True).start()
            else: operation(request)
    finally:
        assistant.stop_loop()


if __name__ == '__main__': main()
