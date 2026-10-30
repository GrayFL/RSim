import asyncio
import os
import signal
import sys
from pathlib import Path
from rsim.core import Component, ComponentError
from .arguments import GeneratedParameters, RosArguments


async def _join_transaction(task):
    """Recover owned resources even when the caller is cancelled repeatedly."""
    cancelled = False
    while not task.done():
        try:
            await asyncio.shield(task)
        except asyncio.CancelledError:
            if task.cancelled():
                raise
            cancelled = True
    return task.result(), cancelled


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
            ros_args=None,
            generated_parameters=None,
        ):
        super().__init__(key=key)
        self.package, self.executable = package, executable
        self.options = parameters if isinstance(parameters, RosArguments) else RosArguments(
            parameters, ros_args, remappings)
        self.parameters = self.options.parameters
        self.generated_parameters = (generated_parameters if isinstance(generated_parameters, GeneratedParameters)
                                     else GeneratedParameters(generated_parameters)
                                     if generated_parameters is not None else None)
        self._parameter_file = None
        self.log_path = log_path
        self.remappings = self.options.remappings
        self.process = self._log = None
        self._device_lease = None
        self._close_task = None

    def configuration(self):
        return (super().configuration(), self.package, self.executable, self.options.signature(),
                self.generated_parameters.digest if self.generated_parameters is not None else None)

    def __getstate__(self):
        state = super().__getstate__()
        state.update(process=None, _log=None, _device_lease=None, _parameter_file=None, _close_task=None)
        return state

    async def open(self):
        from ament_index_python.packages import get_package_prefix
        from rsim.runtime.locks import acquire_device
        executable = Path(
            get_package_prefix(self.package)
            ) / "lib" / self.package / self.executable
        self._close_task = None
        try:
            args = self.options.arguments()  # Recheck user-owned file digests.
            self._device_lease = acquire_device(self.key)
            if self.generated_parameters is not None:
                self._parameter_file = self.generated_parameters.materialize()
                self._parameter_file.validate()
                args = ['--ros-args', '--params-file', str(self._parameter_file.path), '--', *args]
            if self.log_path:
                Path(self.log_path).parent.mkdir(parents=True, exist_ok=True)
                self._log = open(self.log_path, "a")
            launch = asyncio.create_task(asyncio.create_subprocess_exec(
                sys.executable, "-m", "rsim.platforms.linux.exec", str(os.getpid()),
                str(executable), *args,
                stdout=self._log or asyncio.subprocess.DEVNULL,
                stderr=self._log or asyncio.subprocess.DEVNULL))
            self.process, cancelled = await _join_transaction(launch)
            if cancelled:
                raise asyncio.CancelledError
            self.task("driver-health", self.check, hz=20)
        except BaseException:
            await self.close()
            raise

    async def check(self):
        if self.process.returncode is not None:
            raise ComponentError(
                f"{self.package} driver exited: {self.process.returncode}"
                )

    async def close(self):
        if self._close_task is None:
            self._close_task = asyncio.create_task(self._close_resources())
        _, cancelled = await _join_transaction(self._close_task)
        if cancelled:
            raise asyncio.CancelledError

    async def _close_resources(self):
        try:
            if self.process is not None and self.process.returncode is None:
                try:
                    self.process.send_signal(signal.SIGINT)
                except ProcessLookupError:
                    pass
                try:
                    await asyncio.wait_for(self.process.wait(), 5)
                except TimeoutError:
                    try:
                        self.process.kill()
                    except ProcessLookupError:
                        pass
                    await self.process.wait()
        finally:
            if self._log is not None:
                self._log.close()
                self._log = None
            if self._device_lease is not None:
                self._device_lease.close()
                self._device_lease = None
            if self._parameter_file is not None:
                self._parameter_file.close()
                self._parameter_file = None
