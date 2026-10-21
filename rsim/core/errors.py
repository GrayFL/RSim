"""Runtime errors, independent of adapters and transports."""


class ComponentError(RuntimeError):
    pass


class HistoryMiss(LookupError):
    pass


class PortNotBound(ComponentError):
    """A remote port was declared, but not requested by this Runtime."""


class ProviderDisconnected(ComponentError):
    """The leased provider instance ended; explicitly reopen to reconnect."""


class DeploymentError(ComponentError):
    """Structured runtime failure. The original exception is retained as cause."""
    def __init__(self, message, *, phase=None, platform=None, instance_id=None, path=None):
        super().__init__(message)
        self.phase, self.platform = phase, platform
        self.instance_id, self.path = instance_id, str(path) if path is not None else None


class PlatformUnavailable(DeploymentError):
    pass


class StartupFailed(DeploymentError):
    pass


class CleanupFailed(DeploymentError):
    pass


# Compatibility for applications written before the Component/Signal split.
SensorError = ComponentError
