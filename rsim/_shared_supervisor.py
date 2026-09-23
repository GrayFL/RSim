"""Source ownership lasts until the final client socket disconnects."""
import fcntl
import json
import os
from pathlib import Path
import selectors
import shutil
import signal
import socket
import subprocess
import sys


def main():
    directory = Path(sys.argv[1])
    listener = socket.socket(fileno=int(sys.argv[2]))
    initial = socket.socket(fileno=int(sys.argv[3]))
    socket_path, lock_path = Path(sys.argv[4]), Path(sys.argv[5])
    listener.setblocking(False)
    initial.sendall(b"1")
    clients = {initial}
    stopping = False
    lock = lock_path.open("a+")
    worker = None

    def stop(*_):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        worker = subprocess.Popen([
            sys.executable, "-m", "rsim._exec", str(os.getpid()),
            sys.executable, "-m", "rsim._worker", str(directory),
        ], start_new_session=True)
        with selectors.DefaultSelector() as selector:
            selector.register(listener, selectors.EVENT_READ)
            selector.register(initial, selectors.EVENT_READ)
            while not stopping:
                if worker.poll() is not None:
                    shutil.rmtree(directory / "frames", ignore_errors=True)
                    status = directory / "status.json"
                    state = json.loads(status.read_text()) if status.exists() else {}
                    if "error" not in state:
                        temporary = directory / "supervisor-status.tmp"
                        temporary.write_text(json.dumps({"pid": worker.pid,
                            "error": f"worker exited: {worker.returncode}"}))
                        temporary.replace(status)
                for selected, _ in selector.select(timeout=0.1):
                    connection = selected.fileobj
                    if connection is listener:
                        while True:
                            try:
                                client, _ = listener.accept()
                            except BlockingIOError:
                                break
                            try:
                                client.sendall(b"1")
                            except (BrokenPipeError, ConnectionResetError):
                                client.close()
                                continue
                            clients.add(client)
                            selector.register(client, selectors.EVENT_READ)
                    else:
                        try:
                            disconnected = not connection.recv(1)
                        except ConnectionResetError:
                            disconnected = True
                        if disconnected:
                            selector.unregister(connection)
                            clients.remove(connection)
                            connection.close()
                if not clients:
                    # Serialize last-reader shutdown with a new client attaching.
                    # Nonblocking is crucial: an attaching client holds the same
                    # lock while waiting for our connection acknowledgement.
                    try:
                        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    except BlockingIOError:
                        continue
                    listener.close()
                    break
    finally:
        # Also serialize externally requested shutdown against new attachments.
        fcntl.flock(lock, fcntl.LOCK_EX)
        listener.close()
        for client in clients:
            client.close()
        if worker is not None and worker.poll() is None:
            try:
                os.killpg(worker.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                worker.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(worker.pid, signal.SIGKILL)
                worker.wait()
        shutil.rmtree(directory, ignore_errors=True)
        socket_path.unlink(missing_ok=True)
        lock.close()


if __name__ == "__main__":
    main()
