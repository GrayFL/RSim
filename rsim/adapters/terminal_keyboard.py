"""POSIX terminal input for SSH: repeated characters act as short key pulses."""

import asyncio
import math
import os
import sys
import time

from rsim.core.component import PrimaryComponent, ComponentError


class TerminalKeyboard(PrimaryComponent):
    """Read only the foreground terminal; restore its settings on Runtime exit.

    Classic terminals report characters, not key releases. A key expires after
    repeat_timeout without another character. This is an approximation and
    cannot reproduce arbitrary simultaneously held desktop keys.
    """

    def __init__(self, *, fd=None, repeat_timeout=0.18, hz=60):
        if not math.isfinite(repeat_timeout) or not 0.05 <= repeat_timeout <= 0.5:
            raise ValueError("repeat_timeout must be between 0.05 and 0.5 seconds")
        if not math.isfinite(hz) or hz <= 0:
            raise ValueError("hz must be positive")
        super().__init__(history=8)
        self.fd = fd
        self.repeat_timeout, self.hz = repeat_timeout, hz
        self.saved_attributes = None
        self.saved_blocking = None
        self.loop = None

    async def open(self):
        import termios
        import tty

        if self.fd is None:
            self.fd = sys.stdin.fileno()
        if not os.isatty(self.fd):
            raise ComponentError(
                "terminal input requires an interactive TTY; connect with ssh -t"
            )
        self.keys, self.quit, self.brake, self.disconnected = {}, False, False, False
        self.escape = bytearray()
        self.escape_started = 0.0
        self.paste = False
        self.saved_attributes = termios.tcgetattr(self.fd)
        mode = termios.tcgetattr(self.fd)
        tty.cfmakecbreak(mode)  # no echo or line buffering; Ctrl-C stays a signal
        mode[0] &= ~termios.IXON
        termios.tcsetattr(self.fd, termios.TCSAFLUSH, mode)
        self.saved_blocking = os.get_blocking(self.fd)
        os.set_blocking(self.fd, False)
        self.loop = asyncio.get_running_loop()
        self.loop.add_reader(self.fd, self.read)
        self.task("terminal-keyboard", self.sample, hz=self.hz)

    def read(self):
        try:
            data = os.read(self.fd, 4096)
        except BlockingIOError:
            return
        except OSError:
            data = b""
        if not data:
            self.disconnected = self.quit = self.brake = True
            self.keys.clear()
            self.loop.remove_reader(self.fd)
            return
        now = time.monotonic()
        for value in data:
            if self.quit:
                break
            if self.escape:
                self.escape.append(value)
                if len(self.escape) == 2:
                    if value not in (ord("["), ord("O")):
                        self.quit = self.brake = True
                        self.keys.clear()
                        self.escape.clear()
                    continue
                if 0x40 <= value <= 0x7E:
                    # Ignore arrows/function keys rather than treating CSI A/D
                    # as steering. Bracketed paste bodies never become input.
                    if self.escape == b"\x1b[200~":
                        self.paste = True
                        self.keys.clear()
                        self.brake = True
                    elif self.escape == b"\x1b[201~":
                        self.paste = False
                    self.escape.clear()
                elif len(self.escape) > 64:
                    self.quit = self.brake = True
                    self.keys.clear()
                continue
            if value == 27:
                self.escape = bytearray(b"\x1b")
                self.escape_started = now
                continue
            if self.paste:
                continue
            if value in (3, 4) or chr(value).lower() == "q":
                self.quit = self.brake = True
                self.keys.clear()
            elif value == 32:
                self.keys.clear()
                self.brake = True
            elif chr(value).lower() in "wasd":
                key = chr(value).lower()
                self.brake = False
                self.keys.pop({"w": "s", "s": "w", "a": "d", "d": "a"}[key], None)
                self.keys[key] = now + self.repeat_timeout

    async def sample(self):
        now = time.monotonic()
        if self.escape and now - self.escape_started >= 0.05:
            # Escape alone exits; incomplete control sequences also fail closed.
            self.quit = self.brake = True
            self.keys.clear()
        self.keys = {
            key: deadline for key, deadline in self.keys.items() if deadline > now
        }
        await self.publish(
            dict(keys=sorted(self.keys), brake=self.brake, quit=self.quit),
            stamp_ns=time.monotonic_ns(),
            clock="host:monotonic",
        )

    async def close(self):
        import termios

        if self.loop is not None:
            self.loop.remove_reader(self.fd)
        try:
            if self.saved_attributes is not None:
                try:
                    termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.saved_attributes)
                except termios.error:
                    if not self.disconnected:
                        raise
        finally:
            if self.saved_blocking is not None:
                os.set_blocking(self.fd, self.saved_blocking)
