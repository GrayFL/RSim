import asyncio
import os
import signal
import sys
from pathlib import Path
from rsim.core import Component, ComponentError
from .arguments import RosArguments

class Driver(Component):
    """Direct native executable launch, guarded against parent death on Linux."""

    def __init__(
            self,
            package,
            executable,
            parameters=None,
            *,
            key,
            log_path=None,
            remappings=None,
            ros_args=None
        ):
        super().__init__(key=key)
        self.package, self.executable = package, executable
        self.options = parameters if isinstance(parameters, RosArguments) else RosArguments(
            parameters, ros_args, remappings)
        self.parameters = self.options.parameters
        self.log_path = log_path
        self.remappings = self.options.remappings
        self.process = self._log = None
        self._device_lease = None

    def configuration(self):
        return super().configuration(), self.package, self.executable, self.options.signature()

    def __getstate__(self):
        state = super().__getstate__()
        state.update(process=None, _log=None, _device_lease=None)
        return state

    async def open(self):
        from ament_index_python.packages import get_package_prefix
        from rsim.runtime.locks import acquire_device
        executable = Path(
            get_package_prefix(self.package)
            ) / "lib" / self.package / self.executable
        args = [str(executable), *self.options.arguments()]
        self._device_lease = acquire_device(self.key)
        if self.log_path:
            self._log = open(self.log_path, "a")
        self.process = await asyncio.create_subprocess_exec(
            sys.executable,
            "-m",
            "rsim.runtime.exec",
            str(os.getpid()),
            *args,
            stdout=self._log or asyncio.subprocess.DEVNULL,
            stderr=self._log or asyncio.subprocess.DEVNULL
            )
        self.task("driver-health", self.check, hz=20)

    async def check(self):
        if self.process.returncode is not None:
            raise ComponentError(
                f"{self.package} driver exited: {self.process.returncode}"
                )

    async def close(self):
        try:
            if self.process is not None and self.process.returncode is None:
                try:
                    self.process.send_signal(signal.SIGINT)
                except ProcessLookupError:
                    pass
                try:
                    await asyncio.wait_for(self.process.wait(), 5)
                except TimeoutError:
                    self.process.kill()
                    await self.process.wait()
        finally:
            if self._log is not None:
                self._log.close()
                self._log = None
            if self._device_lease is not None:
                self._device_lease.close()
                self._device_lease = None
