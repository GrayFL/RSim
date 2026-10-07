"""SDL window worker. No ROS, chassis client, or command transport imports."""

import asyncio
import json
import math
import os
import signal
import time

from rsim.core.metronome import Metronome


class WindowInput:
    """Focus/brake latch: regaining focus never restores old held keys."""

    def __init__(self):
        self.keys, self.blocked = set(), set()
        self.focused = self.connected = self.quit = False
        self.space = False
        self.brake = True

    def stop(self):
        self.blocked.update(self.keys)
        self.keys.clear()
        self.brake = True

    def focus(self, gained, held=()):
        self.stop()
        self.focused = gained
        self.space = gained and "space" in held
        if gained:
            self.blocked = set(held)

    def connect(self, connected, held=()):
        if connected != self.connected:
            self.stop()
            self.blocked.update(held)
            self.connected = connected
            self.space = "space" in held

    def key(self, key, down, *, repeat=False):
        if not down:
            self.keys.discard(key)
            self.blocked.discard(key)
            if key == "space":
                self.space = False
        elif key == "esc":
            self.quit = True
            self.stop()
        elif key == "space":
            self.space = True
            self.stop()
        elif (
            key in "wasd"
            and len(key) == 1
            and self.focused
            and self.connected
            and not self.quit
            and not repeat
            and not self.space
            and key not in self.blocked
        ):
            self.keys.add(key)
            self.brake = False

    def snapshot(self):
        return dict(
            keys=sorted(self.keys),
            brake=self.brake,
            quit=self.quit,
            focused=self.focused,
            at=time.monotonic(),
        )


class WindowPanel:
    """Small software-rendered dashboard; font fallback is per glyph."""

    background = (15, 22, 32)
    panel = (24, 34, 47)
    foreground = (227, 235, 243)
    muted = (151, 166, 182)
    accent = (86, 211, 178)
    amber = (249, 191, 95)

    def __init__(self, pg, fonts, *, surface=None, update=None):
        self.pg = pg
        self.screen = (
            surface if surface is not None else pg.display.set_mode((760, 480))
        )
        self.update = update or pg.display.update
        paths = [pg.font.match_font(name.strip()) for name in fonts.split(",")]
        self.paths = list(dict.fromkeys(path for path in paths if path)) or [None]
        self.fonts = {
            size: [pg.font.Font(path, size) for path in self.paths]
            for size in (16, 20, 28, 40)
        }
        self.previous = {}
        self.glyphs = {}

    def glyph_font(self, character, size):
        if (character, size) in self.glyphs:
            return self.glyphs[character, size]
        for font in self.fonts[size]:
            metrics = font.metrics(character)[0]
            # Some patched Nerd Fonts contain blank placeholder glyphs for
            # CJK, with a nonzero advance but no ink bounding box.
            if metrics is not None and (character.isspace() or metrics[1] > metrics[0]):
                self.glyphs[character, size] = font
                return font
        self.glyphs[character, size] = self.fonts[size][-1]
        return self.glyphs[character, size]

    def text(self, value, x, y, size=20, color=None):
        # Match each glyph so the Latin font doesn't hide the Chinese fallback.
        for char in str(value):
            font = self.glyph_font(char, size)
            image = font.render(char, True, color or self.foreground)
            self.screen.blit(image, (x, y))
            x += font.size(char)[0]

    def meter(self, label, value, limit, y, unit):
        pg = self.pg
        self.text(label, 310, y, 16, self.muted)
        self.text(f"{value:+.3f} {unit}", 310, y + 21, 28)
        bar = pg.Rect(310, y + 59, 408, 10)
        pg.draw.rect(self.screen, self.background, bar, border_radius=5)
        fraction = max(-1.0, min(1.0, value / max(limit, 1e-9)))
        width = round(abs(fraction) * bar.width / 2)
        start = bar.centerx if fraction >= 0 else bar.centerx - width
        pg.draw.rect(self.screen, self.accent, (start, bar.y, width, bar.height))
        pg.draw.line(
            self.screen,
            self.muted,
            (bar.centerx, bar.y - 3),
            (bar.centerx, bar.bottom + 3),
        )

    def render(self, view, inputs):
        pg = self.pg
        self.screen.fill(self.background)
        self.text("RSIM / 底盘控制", 24, 18, 28)
        connected = view.get("connected", False)
        mode = "ZERO OUTPUT" if view.get("dry_run", True) else "LIVE OUTPUT"
        self.text(
            mode, 544, 26, 20, self.amber if view.get("dry_run", True) else self.accent
        )
        status = "连接中 / CONNECTING"
        if connected:
            status = "已连接 / CONNECTED" if inputs.focused else "失焦制动 / PAUSED"
        self.text(
            status,
            24,
            59,
            16,
            self.accent if connected and inputs.focused else self.amber,
        )
        self.text(str(view.get("endpoint", ""))[:32], 405, 59, 16, self.muted)
        pg.draw.rect(self.screen, self.panel, (24, 98, 260, 256), border_radius=12)
        pg.draw.rect(self.screen, self.panel, (296, 98, 440, 256), border_radius=12)
        for key, x, y in [
            ("w", 124, 115),
            ("a", 57, 179),
            ("s", 124, 179),
            ("d", 191, 179),
        ]:
            active = key in inputs.keys
            pg.draw.rect(
                self.screen,
                self.accent if active else self.background,
                (x, y, 58, 52),
                border_radius=8,
            )
            self.text(
                key.upper(),
                x + 19,
                y + 9,
                28,
                self.background if active else self.foreground,
            )
        color = self.amber if inputs.brake else self.background
        pg.draw.rect(self.screen, color, (57, 248, 192, 40), border_radius=8)
        self.text(
            "SPACE / 制动",
            76,
            257,
            20,
            self.background if inputs.brake else self.foreground,
        )
        self.text("W/S 前后 · A/D 左右", 49, 315, 16, self.muted)
        self.meter(
            "模型线速度 / LINEAR",
            view.get("linear_x", 0),
            view.get("terminal_speed", 0.1),
            114,
            "m/s",
        )
        self.meter(
            "模型角速度 / YAW",
            view.get("angular_z", 0),
            view.get("yaw_limit", 0.15),
            207,
            "rad/s",
        )
        self.text(
            f"虚拟转向 {view.get('steering_deg', 0):+.1f}°", 310, 307, 16, self.muted
        )
        angle = math.radians(view.get("steering_deg", 0))
        center = (676, 316)
        pg.draw.circle(self.screen, self.muted, center, 23, 1)
        pg.draw.line(
            self.screen,
            self.accent,
            center,
            (
                round(center[0] - 20 * math.sin(angle)),
                round(center[1] - 20 * math.cos(angle)),
            ),
            3,
        )
        pose = view.get("pose")
        pose_text = "POSE / 等待位姿"
        if pose is not None:
            pose_text = (
                f"POSE  x {pose[0]:+.3f} m   y {pose[1]:+.3f} m   yaw {pose[2]:+.1f}°"
            )
        self.text(pose_text, 24, 374, 20)
        if view.get("dry_run", True):
            self.text(
                "发送速度始终为零 / commands remain zero", 24, 407, 16, self.amber
            )
        else:
            self.text("速度条显示模型指令，并非实测轮速", 24, 407, 16, self.muted)
        self.text("失焦即制动 · 重新按键继续 · ESC 关闭", 24, 444, 16, self.muted)
        # X forwarding benefits from updating only regions whose displayed
        # values changed, especially while the vehicle is stationary.
        regions = {
            "header": (
                (connected, inputs.focused, mode, view.get("endpoint")),
                (0, 0, 760, 98),
            ),
            "keys": ((tuple(sorted(inputs.keys)), inputs.brake), (24, 98, 260, 256)),
            "linear": (
                (
                    round(view.get("linear_x", 0), 3),
                    round(view.get("terminal_speed", 0.1), 3),
                ),
                (296, 98, 440, 102),
            ),
            "angular": (
                (
                    round(view.get("angular_z", 0), 3),
                    round(view.get("steering_deg", 0), 1),
                    round(view.get("yaw_limit", 0.15), 3),
                ),
                (296, 200, 440, 154),
            ),
            "pose": (pose_text, (0, 354, 760, 50)),
            "footer": (mode, (0, 404, 760, 76)),
        }
        dirty = [
            rect
            for name, (value, rect) in regions.items()
            if self.previous.get(name) != value
        ]
        if not self.previous:
            dirty = [(0, 0, 760, 480)]
        self.previous = {name: value for name, (value, _) in regions.items()}
        if dirty:
            self.update(dirty)


async def window_loop(channel, title, fonts, render_hz):
    os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
    import pygame as pg

    forwarded = None
    try:
        # Avoid SDL choosing Wayland ahead of a forwarded X display.
        if os.environ.get("SSH_CONNECTION") and os.environ.get("DISPLAY"):
            os.environ.setdefault("SDL_VIDEODRIVER", "x11")
        pg.display.init()
        pg.font.init()
        display = os.environ.get("DISPLAY", "")
        if (
            pg.display.get_driver() == "x11"
            and display
            and not display.startswith((":", "unix:"))
        ):
            from .pygame_x11 import X11Surface

            forwarded = X11Surface(pg, title, (760, 480))
            panel = WindowPanel(
                pg, fonts, surface=forwarded.surface, update=forwarded.update
            )
        else:
            panel = WindowPanel(pg, fonts)
            pg.display.set_caption(title)
        pg.key.set_repeat()  # key state comes from down/up, not character repeat
        inputs = WindowInput()
        inputs.focus(bool(pg.key.get_focused()))
        view = {}
        # First presentation can initialize a WSLg compositor / remote surface.
        # Finish it before announcing ready; no controller is running yet.
        panel.render(view, inputs)
        last_parent = last_render = time.monotonic()
        metronome = Metronome(60)
        names = {
            pg.K_w: "w",
            pg.K_a: "a",
            pg.K_s: "s",
            pg.K_d: "d",
            pg.K_SPACE: "space",
            pg.K_ESCAPE: "esc",
        }

        def held():
            pressed = pg.key.get_pressed()
            return [name for key, name in names.items() if pressed[key]]

        while True:
            await metronome.tick()
            for event in pg.event.get():
                if event.type == pg.WINDOWEXPOSED:
                    panel.previous.clear()
                elif event.type in (pg.QUIT, pg.WINDOWCLOSE):
                    inputs.key("esc", True)
                elif event.type in (
                    pg.WINDOWFOCUSLOST,
                    pg.WINDOWMINIMIZED,
                    pg.WINDOWHIDDEN,
                ):
                    inputs.focus(False)
                elif event.type == pg.WINDOWFOCUSGAINED:
                    inputs.focus(True, held())
                elif event.type in (pg.KEYDOWN, pg.KEYUP) and event.key in names:
                    inputs.key(
                        names[event.key],
                        event.type == pg.KEYDOWN,
                        repeat=getattr(event, "repeat", False),
                    )
            for _ in range(64):
                try:
                    view = json.loads(channel.recv(16384))
                except BlockingIOError:
                    break
                last_parent = time.monotonic()
                if view.get("close"):
                    return
                inputs.connect(bool(view.get("connected")), held())
            if time.monotonic() - last_parent > 2:
                return  # parent stopped or died: don't leave an orphan window
            try:
                channel.send(json.dumps(inputs.snapshot()).encode())
            except BlockingIOError:
                pass
            if inputs.quit:
                return
            now = time.monotonic()
            if now - last_render >= 1 / render_hz:
                panel.render(view, inputs)
                last_render = now
    finally:
        if forwarded is not None:
            forwarded.close()
        pg.display.quit()
        pg.font.quit()


def run_window(channel, title, fonts, render_hz):
    # The parent owns Ctrl-C and shuts down commands before the display.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    try:
        asyncio.run(window_loop(channel, title, fonts, render_hz))
    except Exception as error:
        try:
            channel.send(json.dumps({"error": f"pygame window: {error}"}).encode())
        except OSError:
            pass
    finally:
        channel.close()
