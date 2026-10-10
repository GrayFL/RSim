"""Keep a local window alive while replacing failed control connections."""

import asyncio
import logging
import math

from rsim.adapters.pygame_keyboard import PygameKeyboard
from rsim.components.teleoperation import Teleoperation
from rsim.core.component import PrimaryComponent
from rsim.devices import Chassis
from rsim.runtime import Runtime

from .control import transport, until_closed

logger = logging.getLogger(__name__)


class SessionKeyboard(PrimaryComponent):
    """Copy small input frames from the app-owned window into one session.

    The read callback crosses two explicitly separate Runtime lifetimes. Source
    timestamps are preserved; this bridge never owns or restarts the window.
    """

    def __init__(self, read):
        super().__init__(history=8)
        self.read = read
        self.enabled = self.armed = False

    async def open(self):
        self.task("input", self.sample, hz=60)

    async def sample(self):
        frame = await self.read(timeout=.2)
        value = dict(frame.data)
        if value.get("stalled"):
            raise TimeoutError("pygame input temporarily stalled; commands stopped")
        if not self.enabled or not self.armed:
            self.armed = self.enabled and not value["keys"]
            value.update(keys=[], brake=True)
        await self.publish(value, stamp_ns=frame.stamp_ns, clock=frame.clock,
                           received_ns=frame.received_ns)


async def run_pygame(args, parameters, describe_error):
    keys = PygameKeyboard(fonts=args.font, render_hz=args.window_hz,
                          timeout=parameters.max_loop_gap, recover_stalls=True)
    view = dict(dry_run=args.dry_run, connected=False, reconnecting=False,
                endpoint=f"{args.name} / domain {args.domain}")
    active = None

    async def dashboard():
        values = {}
        if active is not None and view["connected"]:
            chassis, control = active
            if control.state.frames:
                values.update(control.state.frames[-1].data)
            if chassis.pose.frames:
                p = chassis.pose.frames[-1].data
                values['pose'] = [float(p.position[0]), float(p.position[1]),
                                  math.degrees(float(p.euler_rad[2]))]
        keys.present(**dict(values, **view))

    async def connections():
        nonlocal active
        attempt, delay = 0, args.reconnect_delay
        while True:
            while keys.stalled:
                await asyncio.sleep(.05)
            attempt += 1
            # Fresh DDS client, clock bound, command epoch and model each time.
            # No commands or held-key state survive a failed connection.
            view.update(connected=False, connection_id=attempt)
            await dashboard()
            logger.info("Connecting to %s domain=%s attempt=%s", args.name, args.domain, attempt)
            chassis = Chassis(args.name, transport=transport(args))
            feed = SessionKeyboard(keys.output.get)
            control = Teleoperation(feed, chassis.velocity, parameters=parameters,
                                    dry_run=args.dry_run)
            try:
                async with Runtime(control):
                    active = (chassis, control)
                    # Only zeros are allowed until fresh pose and a command
                    # acknowledgement prove this new connection is usable.
                    await chassis.pose.get(timeout=5)
                    async with asyncio.timeout(5):
                        await control.acknowledged.wait()
                    feed.enabled = True
                    view.update(connected=True, reconnecting=False, fault=None)
                    delay = args.reconnect_delay
                    await dashboard()
                    print("Connected; " + ("ZERO OUTPUT" if args.dry_run else "LIVE OUTPUT"), flush=True)
                    logger.info("Connection ready attempt=%s; release held keys before driving", attempt)
                    await until_closed(control, control.finished)
                    return
            except Exception as error:
                if keys._failure is not None:
                    raise RuntimeError("window input failed") from keys._failure
                view.update(connected=False, reconnecting=True, fault=describe_error(error))
                logger.exception("Control connection lost; retrying in %.1fs", delay)
            finally:
                # Runtime has cancelled senders and requested zero/release.
                # When unreachable, the provider's original TTL still expires.
                active = None
                view['connected'] = False
                await dashboard()
            await asyncio.sleep(delay)
            delay = min(5, delay * 2)

    keys.present(**view)
    async with Runtime(keys):
        keys.task("dashboard", dashboard, hz=args.window_hz)
        running = asyncio.create_task(connections(), name="control-connections")
        closed = asyncio.create_task(until_closed(keys, keys.quit_requested))
        try:
            done, _ = await asyncio.wait((running, closed), return_when=asyncio.FIRST_COMPLETED)
            for task in done:
                await task
        finally:
            for task in (running, closed):
                task.cancel()
            results = await asyncio.gather(running, closed, return_exceptions=True)
            for result in results:
                if isinstance(result, Exception):
                    logger.error("Connection shutdown: %s", describe_error(result))
    return keys.exit_reason or "window closed"
