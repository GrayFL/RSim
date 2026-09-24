"""Runtime errors, independent of adapters and transports."""


class ComponentError(RuntimeError):
    pass


class HistoryMiss(LookupError):
    pass


# Compatibility for applications written before the Component/Signal split.
SensorError = ComponentError
