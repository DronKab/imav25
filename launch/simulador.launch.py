from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    # gazebo and px4 execution
    # change username
    gz_sim = ExecuteProcess(
        cmd=[
            "/bin/bash", "-lc",
            "cd /home/lucycv_u22/imav25_sim && gz sim indoor.sdf -r"
        ],
        output="screen"
    )

    px4_sitl = ExecuteProcess(
        cmd=[
            "/bin/bash", "-lc",
            "cd /home/lucycv_u22/PX4-Autopilot && HEADLESS=1 make px4_sitl gz_x500"
        ],
        output="screen"
    )

    microxrce_agent = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "--port", "8888"],
        output="screen"
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/pi_camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/pi_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            "/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
        ],
        output="screen"
    )

    px4_driver_node = Node(
        package="imav25",
        executable="px4_driver",
        output="screen"
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[{
            "dev": "/dev/input/js0",  
            "deadzone": 0.05,
            "autorepeat_rate": 20.0
        }]
    )

    aruco_detections_node = Node(
    package="aruco_opencv",
    executable="aruco_tracker_autostart",
    output="screen",
    parameters=[{
        "cam_base_topic":"/camera/image_raw",
        "marker_dict":"5X5_1000"
    }]
)

    return LaunchDescription([
        microxrce_agent,
        gz_sim,
        px4_sitl,
        TimerAction(period=8.0, actions=[gz_bridge]),
        TimerAction(period=10.0, actions=[px4_driver_node, joy_node, aruco_detections_node]),
    ])