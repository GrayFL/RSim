import asyncio
from types import SimpleNamespace

import pytest

from rsim.adapters.ros2 import context


def test_ros_callback_batch_is_bounded_by_time_and_count(monkeypatch):
    clock = [0.]
    monkeypatch.setattr(context.time, 'monotonic', lambda: clock[0])
    ros = context.RosContext(max_callbacks=8, max_poll_s=.002)
    callbacks = []
    def spin_once(*, timeout_sec):
        assert timeout_sec == 0
        callbacks.append(True)
        clock[0] += .0006
    ros.executor = SimpleNamespace(spin_once=spin_once)
    asyncio.run(ros.poll())
    assert len(callbacks) == 4  # Time budget, not an unbounded queue drain.
    callbacks.clear()
    ros.max_poll_s = 1.
    asyncio.run(ros.poll())
    assert len(callbacks) == 8


def test_ros_callback_failure_is_not_swallowed():
    ros = context.RosContext(max_callbacks=8)
    def fail(**kwargs):
        raise RuntimeError('callback failed')
    ros.executor = SimpleNamespace(spin_once=fail)
    with pytest.raises(RuntimeError, match='callback failed'):
        asyncio.run(ros.poll())


@pytest.mark.parametrize('options', [dict(max_callbacks=0), dict(max_callbacks=1.5),
    dict(max_callbacks=True), dict(max_poll_s=0), dict(max_poll_s=float('nan'))])
def test_ros_polling_configuration_rejects_unbounded_options(options):
    with pytest.raises(ValueError):
        context.RosContext(**options)
