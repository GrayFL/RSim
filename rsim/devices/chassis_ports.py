"""Public differential-drive Shared Ports view, distinct from Chassis RPC."""
from rsim.core import Component, VelocityCommand
from rsim.runtime.sharing import SharedComponent, PortSpec
from .ports import ChassisPorts


class ChassisPortsDevice(SharedComponent):
    """Connect to the public subset of a ``chassis-ports-v1`` provider.

    pose is a graphmap Pose in metres (world to base frame). state supplies
    linear_x (m/s), angular_z (rad/s), acceleration (m/s²), yaw_acceleration
    (rad/s²), frame, episode, step_index and pose_kind. pose/state use the same
    source clock/stamp; metadata carries episode/step_index/pose_kind too.
    velocity accepts VelocityCommand under a host-monotonic safety deadline.

    This minimum capability declaration is not a device's complete spec. Each
    concrete Factory owns that declaration, including optional sensors.
    """
    INTERFACE_VERSION = 'chassis-ports-v1'

    def __init__(self, *, key, clock, history=32, max_ttl=.5, transport=None, options=None):
        if not isinstance(clock, str) or not clock:
            raise ValueError('a chassis view requires an explicit source clock')
        super().__init__(key=key, ports={
            'pose': PortSpec(clock=clock, history_capacity=history),
            'state': PortSpec(clock=clock, history_capacity=history),
            'velocity': PortSpec(direction='sink', clock='host:monotonic',
                                 max_ttl=max_ttl, fallback=VelocityCommand()),
        }, interface_version=self.INTERFACE_VERSION, transport=transport, options=options)

    @staticmethod
    def from_component(source):
        """Bind local public ports without starting a Runtime or constructing a source."""
        if not isinstance(source, Component):
            raise TypeError('source must be a Component with pose/state/velocity ports')
        try:
            ports = ChassisPorts(source.pose, source.velocity, source.state)
        except AttributeError as error:
            raise TypeError('source must expose pose/state/velocity ports') from error
        if not ports.pose.clock or ports.pose.clock != ports.state.clock:
            raise ValueError('pose and state must declare the same source clock')
        return ports
