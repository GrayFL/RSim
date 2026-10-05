from rsim.runtime.host import SharedSensor

def Hipnuc(port, *, history=128, transport=None):
    """Connect to an existing IMU provider without importing ROS or pyserial."""
    from pathlib import Path
    port = str(Path(port).expanduser().resolve())
    source = SharedSensor(key="hipnuc:" + port, version="hipnuc-v1", history=history,
                          hz=500, transport=transport, output_name="imu")
    source.imu = source.output
    return source
