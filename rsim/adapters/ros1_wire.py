"""Versioned latest-value files shared by Python 2 ROS1 and modern RSim.

Intentionally Python 2.7 compatible, stdlib only. Atomic rename prevents torn
reads; each stream has a separate mailbox so a map cannot queue behind control.
This is an IPC boundary, not the DDS array-descriptor wire format.
"""
from __future__ import print_function
import json
import os
import re
import uuid

VERSION = 1
MAX_BYTES = 32 * 1024 * 1024


def notification_address(session, channel):
    """Linux abstract socket: PRoot does not translate sendto path addresses."""
    if not re.match(r'^[a-f0-9]{32}$', session) or channel not in ('command', 'updates'):
        raise ValueError('invalid notification address')
    return '\0rsim.' + session + '.' + channel


def path_for(directory, name):
    if not re.match(r'^[a-zA-Z][a-zA-Z0-9_]{0,63}$', name):
        raise ValueError('invalid mailbox name')
    return os.path.join(directory, name + '.json')


def write_packet(directory, name, packet):
    payload = json.dumps(packet, allow_nan=False, separators=(',', ':')).encode('utf8')
    if len(payload) > MAX_BYTES:
        raise ValueError('mailbox exceeds size limit')
    path = path_for(directory, name)
    temporary = path + '.' + uuid.uuid4().hex + '.tmp'
    try:
        fd = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(fd, 'wb') as stream:
            stream.write(payload)
        # POSIX rename replaces atomically; os.replace also works on Windows.
        getattr(os, 'replace', os.rename)(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def read_packet(directory, name):
    try:
        with open(path_for(directory, name), 'rb') as stream:
            payload = stream.read(MAX_BYTES + 1)
    except IOError as error:
        if error.errno == 2:
            return None
        raise
    if len(payload) > MAX_BYTES:
        raise ValueError('mailbox exceeds size limit')
    result = json.loads(payload.decode('utf8'))
    if not isinstance(result, dict):
        raise ValueError('mailbox must contain an object')
    return result
