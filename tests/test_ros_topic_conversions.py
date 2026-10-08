"""Regression for ROS diagnostic byte fields seen on the live chassis."""

from types import SimpleNamespace

from rsim.adapters.ros2.conversions import diagnostics_data


def test_diagnostic_level_accepts_ros2_byte_and_integer():
    statuses = [
        SimpleNamespace(name="motor", level=bytes([level]), message="ok",
                        hardware_id="board", values=[])
        for level in (0, 1, 2, 3)
    ]
    statuses.append(SimpleNamespace(name="legacy", level=1, message="warn",
                                    hardware_id="board", values=[]))
    result = diagnostics_data(SimpleNamespace(status=statuses))
    assert [status["level"] for status in result["statuses"]] == [0, 1, 2, 3, 1]
