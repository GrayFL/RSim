import json
from pathlib import Path
from rsim.runtime.host import SharedSensor


def Hipnuc(
        port=None,
        *,
        mode="serial",
        baudrate=460800,
        frame_id="hipnuc_imu",
        navigation_frame="device_navigation",
        history=128,
        parameters=None,
        ros_args=None,
        transport=None,
        log_path=None
    ):
    """One shared serial device, through Python or the rsim_hipnuc ROS2 node."""
    from rsim.adapters.hipnuc import serial_port
    from rsim.adapters.ros2.arguments import RosArguments
    if mode not in ("serial", "ros2"):
        raise ValueError("IMU mode must be serial or ros2")
    defaults = dict(
        port=port or "",
        baudrate=baudrate,
        frame_id=frame_id,
        navigation_frame=navigation_frame,
        history=history
        )
    values = {**defaults, **(parameters or {})}
    options = None
    if mode == "ros2":
        options = RosArguments(values, ros_args)
        with options.resolve("hipnuc_imu") as node:
            values = {
                name: node.get_parameter(name).value
                for name in node.list_parameters([], depth=0).names
                }
            topic = node.resolve_topic_name("imu/data")
        port = serial_port(values["port"])
        # Freeze auto-discovery before starting a shared worker. Later user ROS
        # overrides still take precedence and resolved to this same port above.
        options.parameters["port"] = port
    else:
        if ros_args:
            raise ValueError("ros_args requires mode='ros2'")
        port = serial_port(values["port"])
        values["port"] = port
    log_path = str(
        Path(log_path).resolve()
        ) if log_path is not None else None

    def factory():
        if mode == "serial":
            from rsim.adapters.hipnuc import SerialIMU
            return SerialIMU(**values)
        from rsim.adapters.ros2 import Driver, RosSensor
        driver = Driver(
            "rsim_hipnuc",
            "serial_node",
            options,
            key="hipnuc-node:" + port,
            log_path=log_path
            )
        # The serial endpoint owns the physical-port lock in both modes.
        return RosSensor(
            topic,
            "imu",
            clock="ros:sim"
            if values.get("use_sim_time") else "ros:system",
            driver=driver,
            history=history,
            hz=500
            )

    signature = options.signature() if options is not None else json.dumps(
        values, sort_keys=True
        )
    source = SharedSensor(
        factory,
        key="hipnuc:" + port,
        version="hipnuc-v1",
        history=history,
        hz=500,
        transport=transport,
        provider_version=mode + ":" + signature,
        output_name="imu"
        )
    source.imu = source.output
    return source
