from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=["MicroXRCEAgent", "udp4","--port","8888"],
            output="log"
        ),
        Node(
            package="imav25",
            executable="px4_driver",
            output="screen"
        ),
        Node(
            package="aruco_opencv",
            executable="aruco_tracker_autostart",
            output="screen",
            parameters=[{
                "cam_base_topic":"/camera/image_raw",
                "marker_dict":"5X5_1000"
            }]
        ),
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
            parameters=[{
                "dev": "/dev/input/js0",  
                "deadzone": 0.05,
                "autorepeat_rate": 20.0
            }]
        ),
        Node(
            package="imav25",
            executable="move_drone",
            output="screen"
        ),
        Node(
            package="imav25",
            executable="tunnel_detect",
            output="screen"
        ), 
        Node(
            package="imav25",
            executable="whiteboard_detect",
            output="screen"
        )
    ])