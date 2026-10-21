"""Internal OS primitives. Importing this package performs no capability probes."""

from .detect import get_platform

__all__ = ['get_platform']
