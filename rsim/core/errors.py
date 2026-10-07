"""Runtime errors, independent of adapters and transports."""


class ComponentError(RuntimeError):
    pass


class HistoryMiss(LookupError):
    pass


class PortNotBound(ComponentError):
    """A remote port was declared, but not requested by this Runtime."""


class ProviderDisconnected(ComponentError):
    """The leased provider instance ended; explicitly reopen to reconnect."""


# Compatibility for applications written before the Component/Signal split.
SensorError = ComponentError
