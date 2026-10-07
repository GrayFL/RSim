"""Assemble a local estimator/controller and publish a named control service."""


def Chassis(
    robot=None,
    *,
    name="chassis",
    simulate=False,
    motion_enabled=None,
    transport=None,
    control=None,
):
    from rsim.adapters.control_service import MotionService

    if simulate:
        if robot is not None:
            raise ValueError("pass a robot or simulate=True")
        from rsim.components.simulated_chassis import SimulatedChassis
        from rsim.components.motion import ChassisController

        source = SimulatedChassis(noise=False, gyro_bias=0.0)
        controller = ChassisController(
            source, motion_enabled=bool(motion_enabled), **(control or {})
        )
        return MotionService(controller, name=name, transport=transport)
    if robot is None:
        raise ValueError("supply a configured local robot or simulate=True")
    if control:
        raise ValueError("configure the supplied robot controller before serving it")
    if (
        motion_enabled is not None
        and bool(motion_enabled) != robot.control.motion_enabled
    ):
        raise ValueError("motion_enabled must match the supplied robot controller")
    return MotionService(
        robot.control, name=name, state=robot.chassis.state, transport=transport
    )
