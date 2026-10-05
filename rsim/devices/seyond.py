from rsim.runtime.host import SharedSensor

def RobinW(ip="192.168.199.97", *, history=8, transport=None):
    source = SharedSensor(
        key=f"robin:{ip}",
        version="robin-v1",
        history=history,
        transport=transport, output_name="points"
        )
    source.points = source.output
    return source
