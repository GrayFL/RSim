"""Same-user local streams: Unix sockets or authenticated Windows loopback.

Windows endpoint records live in the user's private temporary registry. Tokens
authenticate before registry packets are accepted; no executable recipe is sent.
"""
import asyncio
import json
import os
from pathlib import Path
import secrets


def write_endpoint(path, port, token):
    path = Path(path)
    temporary = path.with_suffix('.endpoint.tmp')
    temporary.write_text(json.dumps(dict(protocol=1, host='127.0.0.1', port=port, token=token)), encoding='utf-8')
    temporary.replace(path)


async def open_connection(path, *, limit=65536):
    if os.name != 'nt':
        return await asyncio.open_unix_connection(str(path), limit=limit)
    endpoint = json.loads(Path(path).read_text(encoding='utf-8'))
    if endpoint.get('protocol') != 1 or endpoint.get('host') != '127.0.0.1':
        raise ConnectionError('invalid local endpoint')
    reader, writer = await asyncio.open_connection('127.0.0.1', endpoint['port'], limit=limit)
    try:
        async with asyncio.timeout(3):
            writer.write(endpoint['token'].encode('ascii') + b'\n')
            await writer.drain()
            if await reader.readexactly(1) != b'1':
                raise ConnectionError('local endpoint rejected authentication')
        return reader, writer
    except BaseException:
        writer.close()
        await writer.wait_closed()
        raise


async def start_server(callback, *, path=None, sock=None, limit=65536):
    if os.name != 'nt':
        return await asyncio.start_unix_server(callback, path=path, sock=sock, limit=limit)
    if path is None or sock is not None:
        raise ValueError('Windows local server requires an endpoint record path')
    token = secrets.token_hex(32)
    async def authenticate(reader, writer):
        try:
            async with asyncio.timeout(3):
                line = await reader.readexactly(65)
                if not secrets.compare_digest(line, token.encode() + b'\n'):
                    raise ConnectionError('invalid local token')
                writer.write(b'1')
                await writer.drain()
            await callback(reader, writer)
        except (ConnectionError, asyncio.IncompleteReadError, TimeoutError):
            writer.close()
    server = await asyncio.start_server(authenticate, '127.0.0.1', 0, limit=limit)
    write_endpoint(path, server.sockets[0].getsockname()[1], token)
    return server
