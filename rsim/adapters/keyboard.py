"""pynput keyboard events marshalled from its listener thread into asyncio."""

import asyncio
import time

from rsim.core.component import PrimaryComponent, ComponentError


class PynputKeyboard(PrimaryComponent):
    """Global desktop keys: WASD, space (brake), Escape (close).

    The listener does no I/O or control work. Repeated key-down events are
    idempotent; only the asyncio loop mutates the pressed-key set.
    """

    def __init__(self, *, hz=60, listener_factory=None):
        super().__init__(history=8)
        self.hz, self.listener_factory = hz, listener_factory
        self.listener = None

    def event(self, key, pressed):
        if self._closed or self._closing:
            return
        if key in {"w", "a", "s", "d", "space"}:
            if pressed:
                self.keys.add(key)
            else:
                self.keys.discard(key)
        if key == "esc" and pressed:
            self.quit = True
            self.keys.clear()

    async def open(self):
        self.keys, self.quit = set(), False
        loop = asyncio.get_running_loop()

        def callback(key, pressed):
            name = getattr(key, "char", None) or getattr(key, "name", "")
            if not loop.is_closed():
                loop.call_soon_threadsafe(self.event, name.lower(), pressed)

        factory = self.listener_factory
        if factory is None:
            try:
                from pynput.keyboard import Listener
            except ImportError as error:
                raise ComponentError(
                    "pynput needs an accessible desktop display; use --input terminal for SSH"
                ) from error
            factory = Listener
        self.listener = factory(
            on_press=lambda key: callback(key, True),
            on_release=lambda key: callback(key, False),
        )
        self.listener.start()
        self.task("keyboard", self.sample, hz=self.hz)

    async def sample(self):
        if not self.listener.is_alive():
            raise ComponentError("keyboard listener stopped")
        await self.publish(
            dict(
                keys=sorted(self.keys & set("wasd")),
                brake="space" in self.keys,
                quit=self.quit,
            ),
            stamp_ns=time.monotonic_ns(),
            clock="host:monotonic",
        )

    async def close(self):
        if self.listener is not None:
            self.listener.stop()
            await asyncio.to_thread(self.listener.join, 1.0)
            self.listener = None
