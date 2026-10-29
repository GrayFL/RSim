"""RPC control-service recipes around explicitly supplied local resources."""
from dataclasses import dataclass
from typing import ClassVar

from rsim.runtime.sharing import PortSpec, component_ports, validate_component_ports
from .configuration import fingerprint, freeze_fields, plain


@dataclass(frozen=True)
class ChassisFactory:
    name: str = 'chassis'
    simulate: bool = False
    motion_enabled: bool | None = None
    control: object = None
    FACTORY_VERSION: ClassVar[str] = 'chassis-service-factory-v1'

    def __post_init__(self):
        from rsim.runtime.service import topic_prefix
        topic_prefix(self.name)
        freeze_fields(self)

    @staticmethod
    def port_specs(*, pose):
        """Declare local telemetry using the explicitly injected pose source.

        The source is inspected only for this call, never retained in the recipe.
        Motion remains the existing RPC protocol, not a shared command sink.
        """
        return {'pose': component_ports(pose.producer)[pose.name],
                'state': PortSpec(history_capacity=32, clock='host:monotonic')}

    def fingerprint(self):
        return fingerprint(self)

    def device(self, *, transport=None):
        from rsim.devices.chassis import Chassis as ChassisDevice
        return ChassisDevice(self.name, transport=transport)

    def build(self, robot=None, *, transport=None, pose=None, velocity=None, state=None):
        name, simulate, motion_enabled = self.name, self.simulate, self.motion_enabled
        control = plain(self.control)
        from rsim.adapters.control_service import MotionService
        from rsim.components.motion import ChassisController

        def service_for(controller, state):
            service = MotionService(controller, name=name, state=state, transport=transport)
            validate_component_ports(service, self.port_specs(pose=controller.pose))
            return service

        if pose is not None or velocity is not None:
            if robot is not None or simulate:
                raise ValueError('pass pose/velocity ports, a robot, or simulate=True')
            controller = ChassisController(pose=pose, velocity=velocity,
                motion_enabled=bool(motion_enabled), **(control or {}))
            return service_for(controller, state)
        if state is not None:
            raise ValueError('state is only used with explicit pose/velocity ports')

        if simulate:
            if robot is not None:
                raise ValueError("pass a robot or simulate=True")
            from pyrobo.rsim.drivers import MemoryChassisFactory
            source = MemoryChassisFactory(motion_enabled=bool(motion_enabled)).build()
            controller = ChassisController(
                pose=source.pose, velocity=source.velocity,
                motion_enabled=bool(motion_enabled), **(control or {})
            )
            return service_for(controller, source.state)
        if robot is None:
            raise ValueError("supply a configured local robot or simulate=True")
        if control:
            raise ValueError("configure the supplied robot controller before serving it")
        if (
            motion_enabled is not None
            and bool(motion_enabled) != robot.control.motion_enabled
        ):
            raise ValueError("motion_enabled must match the supplied robot controller")
        service = service_for(robot.control, robot.chassis.state)
        scan = getattr(robot, 'scan', None)
        if scan is not None:
            service.dependencies += (scan.producer,)
        return service


def Chassis(robot=None, *, name='chassis', simulate=False, motion_enabled=None,
            transport=None, control=None, pose=None, velocity=None, state=None):
    """Compatibility constructor; resources are injected into the recipe build."""
    return ChassisFactory(name, simulate, motion_enabled, control).build(
        robot, transport=transport, pose=pose, velocity=velocity, state=state)
