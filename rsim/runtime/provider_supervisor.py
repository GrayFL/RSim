"""Independent source-lease supervisor; stays responsive if the graph blocks.

Only small versioned registry packets pass through this proxy. DDS descriptors
and arrays travel directly between the exporting placement and clients.
"""
import os
from pathlib import Path
import selectors
import shutil
import signal
import socket
import subprocess
import sys


def main():
    directory, listener_fd, initial_fd, lock_fd, path = sys.argv[1:]
    listener = socket.socket(fileno=int(listener_fd))
    initial = socket.socket(fileno=int(initial_fd))
    backend = socket.socket(socket.AF_UNIX)
    backend_path = str(Path(directory) / 'registry.sock')
    backend.bind(backend_path)
    backend.listen(128)
    stopping = False
    worker = None
    peers, buffers, clients = {}, {}, set()
    selector = selectors.DefaultSelector()

    def terminate(*_):
        nonlocal stopping
        stopping = True

    def pair(client):
        upstream = socket.socket(socket.AF_UNIX)
        upstream.connect(backend_path)
        clients.add(client)
        peers[client], peers[upstream] = upstream, client
        for endpoint in (client, upstream):
            endpoint.setblocking(False)
            buffers[endpoint] = bytearray()
            selector.register(endpoint, selectors.EVENT_READ)

    def close_pair(endpoint):
        other = peers.get(endpoint)
        for item in (endpoint, other):
            if item is not None and item in peers:
                selector.unregister(item)
                peers.pop(item)
                buffers.pop(item)
                clients.discard(item)
                item.close()

    signal.signal(signal.SIGTERM, terminate)
    signal.signal(signal.SIGINT, terminate)
    try:
        worker = subprocess.Popen([sys.executable, '-m', 'rsim.runtime.exec', str(os.getpid()),
            sys.executable, '-m', 'rsim.runtime.shared_provider_worker', directory, str(backend.fileno())],
            pass_fds=(backend.fileno(),), start_new_session=True)
        listener.setblocking(False)
        selector.register(listener, selectors.EVENT_READ)
        pair(initial)
        backend.close()
        while clients and not stopping and worker.poll() is None:
            for key, events in selector.select(.1):
                endpoint = key.fileobj
                if endpoint is listener:
                    client, _ = listener.accept()
                    pair(client)
                    continue
                if endpoint not in peers:
                    continue
                try:
                    if events & selectors.EVENT_READ:
                        packet = endpoint.recv(65536)
                        if not packet:
                            close_pair(endpoint)
                            continue
                        other = peers[endpoint]
                        buffers[other].extend(packet)
                        if len(buffers[other]) > 131072:
                            close_pair(endpoint)
                            continue
                        selector.modify(other, selectors.EVENT_READ | selectors.EVENT_WRITE)
                    if events & selectors.EVENT_WRITE and buffers[endpoint]:
                        count = endpoint.send(buffers[endpoint])
                        del buffers[endpoint][:count]
                        if not buffers[endpoint]:
                            selector.modify(endpoint, selectors.EVENT_READ)
                except BlockingIOError:
                    pass
                except (ConnectionError, OSError):
                    close_pair(endpoint)
    finally:
        listener.close()
        backend.close()
        for endpoint in tuple(peers):
            close_pair(endpoint)
        selector.close()
        if worker is not None and worker.poll() is None:
            worker.terminate()
            try:
                worker.wait(5)
            except subprocess.TimeoutExpired:
                worker.kill()
                worker.wait()
        shutil.rmtree(directory, ignore_errors=True)
        Path(path).unlink(missing_ok=True)
        os.close(int(lock_fd))


if __name__ == '__main__':
    main()
