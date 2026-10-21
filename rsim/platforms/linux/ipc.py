"""Same-user Unix streams in a validated private directory."""
import asyncio
import os
import socket
import struct
from contextlib import contextmanager
from pathlib import Path


def endpoint_identity(path):
    info = Path(path).lstat()
    return str(info.st_dev), str(info.st_ino)


@contextmanager
def address(path):
    """Hold the actual registry directory while using a short AF_UNIX address."""
    path = Path(path).absolute()
    descriptor = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
    try:
        # This binds in the configured directory, without the 108-byte limit on
        # the absolute pathname or an alternate discovery namespace.
        yield f'/proc/self/fd/{descriptor}/{path.name}'
    finally:
        os.close(descriptor)


def same_user(writer):
    peer = writer.get_extra_info('socket')
    _, uid, _ = struct.unpack('3i', peer.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, 12))
    return uid == os.getuid()


async def open_connection(path, *, limit=65536):
    with address(path) as endpoint:
        reader, writer = await asyncio.open_unix_connection(endpoint, limit=limit)
    if not same_user(writer):
        writer.close()
        await writer.wait_closed()
        raise PermissionError(f'IPC peer belongs to another user: {path}')
    return reader, writer


async def start_server(callback, *, path=None, sock=None, limit=65536):
    async def authenticate(reader, writer):
        if not same_user(writer):
            writer.close()
            await writer.wait_closed()
            return
        await callback(reader, writer)
    if sock is None:
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            with address(path) as endpoint:
                sock.bind(endpoint)
        except BaseException:
            sock.close()
            raise
    try:
        return await asyncio.start_unix_server(authenticate, sock=sock, limit=limit)
    except BaseException:
        sock.close()
        raise
