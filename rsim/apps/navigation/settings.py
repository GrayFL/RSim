"""Versioned operator settings. Credentials and ROS transport stay in the base file."""
from copy import deepcopy
from dataclasses import asdict
import json
import os
from pathlib import Path
import re

from ...components.vehicle import TowingParameters
from ...components.towing_geometry import BodyFootprint

MODES = ('towing', 'differential')
RESTART_FIELDS = ('profiles', 'permissions')
KEYS = dict(forward='KeyW', backward='KeyS', left='KeyA', right='KeyD', stop='Space')


def defaults(config):
    vehicle = asdict(TowingParameters(**config.get('vehicle', {})))
    values = dict(profiles={mode: deepcopy(vehicle) for mode in MODES},
                manual={'towing': dict(y_mode='acceleration', x_mode='angle_target', angle_reference='steering', release_mode='coast'),
                        'differential': dict(y_mode='velocity_target', x_mode='velocity_target', angle_reference='heading', release_mode='coast')},
                keyboard=KEYS.copy(), control_mode=config.get('control_mode', 'exclusive'), touch_mode='buttons', drive_mode=config.get('drive_mode', 'towing'),
                footprint=asdict(BodyFootprint(**config.get('footprint', {}))),
                speed_limits=dict(navigation=1., manual=1., angular=.8, handles=True),
                permissions=dict(motion=config.get('enable_motion', False),
                                 localization=config.get('ros', {}).get('allow_localization', False),
                                 mapping=config.get('allow_mapping', False), pedal=config.get('allow_pedal', False),
                                 audio=config.get('allow_audio', False)))
    for mode in MODES:
        values['profiles'][mode].update(config.get('profiles', {}).get(mode, {}))
        values['manual'][mode].update(config.get('manual', {}).get(mode, {}))
    values['keyboard'].update(config.get('keyboard', {}))
    values['speed_limits'].update(config.get('speed_limits', {}))
    values['touch_mode'] = config.get('touch_mode', 'buttons')
    return values


def validate(values):
    expected = set(defaults({}))
    if not isinstance(values, dict) or set(values) != expected: raise ValueError('invalid settings fields')
    data = deepcopy(values)
    for key in ('profiles', 'manual', 'keyboard', 'footprint', 'speed_limits', 'permissions'):
        if not isinstance(data[key], dict): raise ValueError('expected settings object: ' + key)
    if set(data['profiles']) != set(MODES) or set(data['manual']) != set(MODES): raise ValueError('two model profiles required')
    for mode in MODES:
        try: p = TowingParameters(**data['profiles'][mode])
        except (TypeError, ValueError) as error: raise ValueError('invalid ' + mode + ' parameters: ' + str(error)) from error
        data['profiles'][mode] = asdict(p)
        axes = data['manual'][mode]
        if not isinstance(axes, dict) or set(axes) != {'x_mode', 'y_mode', 'angle_reference', 'release_mode'}: raise ValueError('invalid manual axes; reload settings')
        if axes['release_mode'] not in ('coast', 'target_zero'): raise ValueError('invalid manual release mode')
        if axes['angle_reference'] not in ('steering', 'heading'): raise ValueError('invalid angle reference')
        if axes['y_mode'] not in ('velocity_target', 'acceleration'): raise ValueError('Y supports linear velocity or acceleration')
        if axes['x_mode'] not in ('angle_target', 'velocity_target', 'acceleration'): raise ValueError('invalid X axis mode')
    keyboard = data['keyboard']
    if any(not isinstance(k, str) or not re.fullmatch(r'Key[A-Z]|Digit[0-9]|Arrow(Up|Down|Left|Right)|Space|Escape', k) for k in keyboard.values()):
        raise ValueError('unsupported keyboard code')
    if set(keyboard) != set(KEYS) or len(set(keyboard.values())) != len(KEYS): raise ValueError('five distinct keyboard bindings required')
    if data['touch_mode'] not in ('buttons', 'joystick') or data['drive_mode'] not in MODES: raise ValueError('invalid control mode')
    if data['control_mode'] not in ('exclusive', 'shared'): raise ValueError('invalid control ownership mode')
    try: data['footprint'] = asdict(BodyFootprint(**data['footprint']))
    except (TypeError, ValueError) as error: raise ValueError('invalid footprint: ' + str(error)) from error
    limits = data['speed_limits']
    if set(limits) != {'navigation', 'manual', 'angular', 'handles'} or type(limits['handles']) is not bool: raise ValueError('invalid speed settings')
    for key in ('navigation', 'manual', 'angular'):
        upper = .8 if key == 'angular' else 1.6
        if type(limits[key]) not in (int, float) or not 0 < limits[key] <= upper: raise ValueError('invalid speed limit: ' + key)
    if set(data['permissions']) != {'motion', 'localization', 'mapping', 'pedal', 'audio'} or any(type(v) is not bool for v in data['permissions'].values()):
        raise ValueError('permissions must be boolean')
    return data


def apply_config(config, values):
    config.update(drive_mode=values['drive_mode'], vehicle=deepcopy(values['profiles'][values['drive_mode']]),
                  profiles=deepcopy(values['profiles']), manual=deepcopy(values['manual']),
                  keyboard=deepcopy(values['keyboard']), touch_mode=values['touch_mode'], control_mode=values['control_mode'],
                  footprint=deepcopy(values['footprint']), speed_limits=deepcopy(values['speed_limits']),
                  enable_motion=values['permissions']['motion'])
    config.setdefault('ros', {})['allow_localization'] = values['permissions']['localization']
    config['ros']['profiles'] = deepcopy(values['profiles'])
    for name in ('mapping', 'pedal', 'audio'): config['allow_' + name] = values['permissions'][name]
    return config


class SettingsStore:
    def __init__(self, path, config):
        self.path = Path(path)
        self.revision = 0
        values = defaults(config)
        values['speed_limits'].update(config.get('speed_limits', {}))
        legacy = self.path.with_name('speed-settings.json')
        if legacy.exists(): values['speed_limits'].update(json.loads(legacy.read_text(encoding='utf8')))
        if self.path.exists():
            document = json.loads(self.path.read_text(encoding='utf8'))
            if document.get('schema') != 1: raise ValueError('unsupported settings schema')
            self.revision, values = document['revision'], document['values']
            # Schema 1 predates configurable ownership. Only persisted legacy
            # files migrate; outdated browser saves must reload the new fields.
            values.setdefault('control_mode', 'exclusive')
            for axes in values.get('manual', {}).values():
                axes.setdefault('release_mode', 'coast')
        self.values = validate(values)
        self.active = deepcopy(self.values)

    def view(self):
        return dict(schema=1, revision=self.revision, values=deepcopy(self.values), active=deepcopy(self.active),
                    restart_required=[key for key in RESTART_FIELDS if self.values[key] != self.active[key]],
                    restart_fields=list(RESTART_FIELDS),
                    live_fields=[key for key in self.values if key not in RESTART_FIELDS])

    def check(self, values, revision):
        if type(revision) is not int or revision != self.revision: raise ValueError('settings changed; reload before saving')
        return validate(values)

    def live(self, values):
        return {k: deepcopy(self.active[k] if k in RESTART_FIELDS else v) for k, v in values.items()}

    def save(self, values, revision):
        values = self.check(values, revision)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix('.next')
        with temporary.open('w', encoding='utf8') as stream:
            json.dump(dict(schema=1, revision=self.revision + 1, values=values), stream, ensure_ascii=False, indent=2)
            stream.flush()
            os.fsync(stream.fileno())
        temporary.replace(self.path)
        self.values, self.active = values, self.live(values)
        self.revision += 1
