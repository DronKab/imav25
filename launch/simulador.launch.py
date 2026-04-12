from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart

def generate_launch_description():

    # execute Gazebo
    # change the username
    gz_sim = ExecuteProcess(
        cmd=[
            "/bin/bash", "-lc",
            "cd /home/lucycv_u22/imav25_sim && gz sim indoor.sdf -r"
        ],
        output="screen"
    )

    # execute PX4 SITL 
    # change the username
    px4_sitl = ExecuteProcess(
        cmd=[
            "/bin/bash", "-lc",
            "cd /home/lucycv_u22/PX4-Autopilot && HEADLESS=1 make px4_sitl gz_x500"
        ],
        output="screen"
    )

    # Micro XRCE Agent
    microxrce_agent = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "--port", "8888"],
        output="screen"
    )

    # run Gazebo Bridge 
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

    # run px4_driver node 
    px4_driver_node = Node(
        package="imav25",
        executable="px4_driver",
        output="screen"
    )

    # run joy_node for move_drone
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

    # run aruco_tracker node
    aruco_detections_node = Node(
        package="aruco_opencv",
        executable="aruco_tracker_autostart",
        output="screen",
        parameters=[{
            "cam_base_topic": "/camera/image_raw",
            "marker_dict": "5X5_1000"
        }]
    )

    # Gazebo starts and after 7s PX4 starts
    start_px4 = RegisterEventHandler(
        OnProcessStart(
            target_action=gz_sim,
            on_start=[
                TimerAction(period=7.0, actions=[px4_sitl])
            ]
        )
    )

    # despues de PX4 + 3s inician el bridge y el Agent
    start_bridge_agent = RegisterEventHandler(
        OnProcessStart(
            target_action=px4_sitl,
            on_start=[
                TimerAction(period=3.0, actions=[gz_bridge, microxrce_agent])
            ]
        )
    )

    # despues del Agent + 2s inicia el px4_driver
    start_driver = RegisterEventHandler(
        OnProcessStart(
            target_action=microxrce_agent,
            on_start=[
                TimerAction(period=2.0, actions=[px4_driver_node])
            ]
        )
    )

    # despues del driver  +3s inicia el joy_node 
    start_joy = RegisterEventHandler(
        OnProcessStart(
            target_action=px4_driver_node,
            on_start=[
                TimerAction(period=3.0, actions=[joy_node])
            ]
        )
    )

    # despues del joy_node + 3s inicia el aruco_tracker
    start_aruco = RegisterEventHandler(
        OnProcessStart(
            target_action=joy_node,
            on_start=[
                TimerAction(period=3.0, actions=[aruco_detections_node])
            ]
        )
    )

    return LaunchDescription([
        gz_sim,
        start_px4,
        start_bridge_agent,
        start_driver,
        start_joy,
        start_aruco,
    ])