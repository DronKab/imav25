from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=["MicroXRCEAgent", "serial", "--dev", "/dev/serial0", "-b", "921600"],
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
                "cam_base_topic":"/pi_camera/image_raw",
                "marker_dict":"5X5_1000",
                "image_sub_compressed": True
            }]
        ),
        Node(
            package="v4l2_camera",
            executable="v4l2_camera_node",
            output="screen",
            remappings=[
                ("__ns","/pi_camera")
            ],
            parameters=[{
                "video_device":"/dev/video2",
                "image_size":[320, 240]
            }]
        )
    ])
