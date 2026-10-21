"""Explicit dispatch: unsupported hosts never silently use Linux primitives."""
from functools import lru_cache
import importlib
import sys

from .contracts import Platform
from .errors import CapabilityUnavailable


@lru_cache(maxsize=2)
def _load(name):
    if name not in ('linux', 'win32'):
        raise CapabilityUnavailable('platform', f'unsupported process/shared-file platform: {name}')
    package = 'linux' if name == 'linux' else 'windows'
    modules = {part: importlib.import_module(f'rsim.platforms.{package}.{part}')
               for part in ('identity', 'process', 'ipc', 'locks', 'files', 'paths')}
    return Platform(name, **modules, os_enforced_tree_containment=name == 'win32')


def get_platform():
    return _load(sys.platform)
