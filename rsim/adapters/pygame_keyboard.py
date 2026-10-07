"""Window keyboard input; SDL rendering is isolated from the control loop."""

import asyncio
import json
import math
import multiprocessing
import socket
import time

from rsim.core.component import ComponentError, PrimaryComponent

DEFAULT_FONTS = "Inconsolata,Sarasa Mono SC"


class PygameKeyboard(PrimaryComponent):
    """A local SDL window, also usable through SSH X forwarding.

    Window input expires independently of DDS leases. IPC uses bounded,
    nonblocking datagrams; a slow display never queues unbounded telemetry.
    The window must run in its child's main thread, not an asyncio worker thread.
    """

    process_local = True

    def __init__(
        self,
        *,
        title="RSim | Chassis control",
        fonts=DEFAULT_FONTS,
        render_hz=20,
        timeout=0.2,
    ):
        super().__init__(history=8)
        if not math.isfinite(render_hz) or not 1 <= render_hz <= 60:
            raise ValueError("window rate must be between 1 and 60 Hz")
        if not math.isfinite(timeout) or not 0.05 <= timeout <= 0.5:
            raise ValueError("window input timeout must be between 0.05 and 0.5 s")
        self.title, self.fonts = title, fonts
        self.render_hz, self.timeout = render_hz, timeout
        self.process = self.channel = None
        self.finished = asyncio.Event()
        self.view = {}

    def present(self, **values):
        """Supply display-only values; these never become commands."""
        self.view = values

    async def open(self):
        from .pygame_window import run_window

        self.finished.clear()
        self.packet = None
        self.channel, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_DGRAM)
        for channel in (self.channel, child):
            channel.setblocking(False)
            channel.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 16384)
        self.process = multiprocessing.get_context("spawn").Process(
            target=run_window,
            args=(child, self.title, self.fonts, self.render_hz),
            name="rsim:pygame",
            daemon=True,
        )
        try:
            self.process.start()
        finally:
            child.close()
        async with asyncio.timeout(10):
            while self.packet is None:
                self.receive()
                await asyncio.sleep(0.01)
        self.task("window-input", self.sample, hz=60)

    def receive(self):
        for _ in range(64):
            try:
                packet = json.loads(self.channel.recv(16384))
            except BlockingIOError:
                break
            if "error" in packet:
                raise ComponentError(packet["error"])
            self.packet = packet
            if packet["quit"]:
                self.finished.set()
        if not self.process.is_alive() and not self.finished.is_set():
            raise ComponentError("pygame window exited")

    async def sample(self):
        self.receive()
        packet = self.packet
        if not packet["quit"] and time.monotonic() - packet["at"] > self.timeout:
            raise ComponentError("pygame window input stalled")
        await self.publish(
            {key: packet[key] for key in ("keys", "brake", "quit", "focused")},
            stamp_ns=int(packet["at"] * 1e9),
            clock="host:monotonic",
        )
        if packet["quit"]:
            return
        try:
            self.channel.send(json.dumps(self.view, allow_nan=False).encode())
        except BlockingIOError:
            pass  # keep the control loop independent of a blocked display

    async def close(self):
        self.finished.set()
        if self.channel is not None:
            try:
                self.channel.send(b'{"close":true}')
            except OSError:
                pass
        if self.process is not None and self.process.pid is not None:
            await asyncio.to_thread(self.process.join, 0.4)
            if self.process.is_alive():
                self.process.terminate()
                await asyncio.to_thread(self.process.join, 0.4)
            if self.process.is_alive():
                self.process.kill()
                await asyncio.to_thread(self.process.join, 0.4)
            self.process.close()
        if self.channel is not None:
            self.channel.close()
        self.process = self.channel = None
