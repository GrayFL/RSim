"""Notebook dashboard using the same pynput client as the CLI.

Keyboard events come from the kernel's desktop session, not browser DOM events.
"""

import asyncio

from rsim.adapters.keyboard import PynputKeyboard
from rsim.components.teleoperation import Teleoperation
from rsim.devices import Chassis
from rsim.runtime import Runtime
from .control import vehicle_parameters, until_closed


class KeyboardDashboard:
    def __init__(self, config, *, name="chassis", transport=None, dry_run=True):
        import ipywidgets as widgets

        self.chassis = Chassis(name, transport=transport)
        self.keyboard = PynputKeyboard()
        self.control = Teleoperation(
            self.keyboard,
            self.chassis.velocity,
            parameters=vehicle_parameters(config),
            dry_run=dry_run,
        )
        self.readout = widgets.HTML("Waiting for start")
        self.brake = widgets.Button(description="Brake")
        self.finish = widgets.Button(
            description="Stop and close", button_style="danger"
        )
        self.widget = widgets.VBox(
            [
                widgets.HTML("W/S throttle · A/D yaw · Space brake · Escape close"),
                self.readout,
                widgets.HBox([self.brake, self.finish]),
            ]
        )
        self.finish.on_click(lambda _: self.control.finished.set())
        self.brake.on_click(lambda _: self.keyboard.event("space", True))
        self.task = None

    async def start(self):
        if self.task is not None:
            raise RuntimeError("dashboard already started")
        self.started = asyncio.Event()
        self.task = asyncio.create_task(self.run(), name="rsim:notebook-keyboard")
        waiter = asyncio.create_task(self.started.wait())
        try:
            done, _ = await asyncio.wait(
                [self.task, waiter], return_when=asyncio.FIRST_COMPLETED
            )
            if self.task in done:
                self.task.result()
        finally:
            waiter.cancel()
            await asyncio.gather(waiter, return_exceptions=True)
        return self.widget

    async def display_state(self):
        previous = 0
        while True:
            frame = await self.control.state.get(after=previous)
            previous = frame.sequence
            s = frame.data
            self.readout.value = (
                f"<pre>v {s['linear_x']:+.3f} m/s    yaw {s['angular_z']:+.3f} rad/s\n"
                f"steering {s['steering_deg']:+.1f}° / ±{s['steering_limit_deg']:.1f}°\n"
                f"terminal speed {s['terminal_speed']:.3f} m/s    "
                f"{'ZERO OUTPUT' if s['dry_run'] else 'LIVE OUTPUT'}</pre>"
            )
            await asyncio.sleep(0.1)

    async def run(self):
        try:
            async with Runtime(self.control):
                self.started.set()
                view = asyncio.create_task(self.display_state())
                try:
                    await until_closed(self.control, self.control.finished)
                finally:
                    view.cancel()
                    await asyncio.gather(view, return_exceptions=True)
        except BaseException as error:
            self.readout.value = "Closed: " + type(error).__name__
            raise
        else:
            self.readout.value = "Closed; zero command sent"

    async def close(self):
        self.control.finished.set()
        if self.task is not None:
            await self.task
