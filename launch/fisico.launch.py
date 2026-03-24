from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([

        ExecuteProcess(
            cmd=["MicroXRCEAgent", "serial", "--dev", "/dev/ttyAMA0", "-b", "921600"],
            output="log"
        ),

        Node(
            package="imav25",
            executable="px4_driver",
            output="screen"
        ),

        # OAK-D: pipeline YOLO + imagen + camera_info en un solo nodo
        Node(
            package="imav25",
            executable="vision_yolo_node",
            name="vision_yolo_node",
            output="screen"
        ),

        # Detección de ArUcos — escucha /oak/rgb y /oak/camera_info
        Node(
            package="aruco_opencv",
            executable="aruco_tracker_autostart",
            name="aruco_tracker",
            output="screen",
            parameters=[{
                "cam_base_topic":       "oak",    # genera /oak/rgb y /oak/camera_info
                "marker_size":          0.10,
                "marker_dict":          "5X5_1000",
                "image_sub_compressed": False,
            }]
        ),
    ])