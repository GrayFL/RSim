"""Explicit clock domains and affine conversions; no inferred clock equality."""
from dataclasses import dataclass
from fractions import Fraction


@dataclass(frozen=True)
class ClockDomain:
    name: str

    def __post_init__(self):
        if not isinstance(self.name, str) or not self.name:
            raise ValueError("clock name must be nonempty")


def clock_name(value):
    return value.name if isinstance(value, ClockDomain) else ClockDomain(value).name


@dataclass(frozen=True)
class ClockTransform:
    """target_ns = source_ns * rate + offset_ns (explicitly calibrated).

    Use Fraction for drift calibration without losing integer timestamp precision.
    The caller is responsible for calibration and its uncertainty/tolerance.
    """
    source: str | ClockDomain
    target: str | ClockDomain
    offset_ns: int = 0
    rate: Fraction = Fraction(1)

    def __post_init__(self):
        object.__setattr__(self, "source", clock_name(self.source))
        object.__setattr__(self, "target", clock_name(self.target))
        object.__setattr__(self, "rate", Fraction(self.rate))
        if self.rate <= 0 or not isinstance(self.offset_ns, int):
            raise ValueError("clock transform needs a positive rate and integer offset")

    def convert(self, stamp_ns, *, clock):
        if clock_name(clock) != self.source:
            raise ValueError(f"clock transform expects {self.source}, received {clock}")
        return round(stamp_ns * self.rate) + self.offset_ns
