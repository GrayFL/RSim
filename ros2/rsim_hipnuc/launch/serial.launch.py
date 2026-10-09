from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {
        "port": "",
        "baudrate": "115200",
        "frame_id": "hipnuc_imu",
        "topic": "/imu/data"
        }
    declarations = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in defaults.items()
        ]
    node = Node(
        package="rsim_hipnuc",
        executable="serial_node",
        output="screen",
        parameters=[{
            "port":
                LaunchConfiguration("port"),
            "baudrate":
                ParameterValue(
                    LaunchConfiguration("baudrate"), value_type=int
                    ),
            "frame_id":
                LaunchConfiguration("frame_id")
            }],
        remappings=[("imu/data", LaunchConfiguration("topic"))]
        )
    return LaunchDescription([*declarations, node])
