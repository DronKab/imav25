from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('x500_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'x500.urdf')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # Ventana 1: Gazebo + PX4
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'bash', '-c',
                 'cd ~/imav25_sim && gz sim indoor.sdf -r & '
                 'cd ~/PX4-Autopilot && HEADLESS=1 make px4_sitl gz_x500; exec bash'],
            output='screen'
        ),

        # Ventana 2: Todos los bridges
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'bash', '-c',
                 'ros2 run ros_gz_bridge parameter_bridge '
                 '/world/indoor/model/x500_0/link/lidar_link/sensor/lidar_2d_v2/scan'
                 '@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan '
                 '--ros-args -r '
                 '/world/indoor/model/x500_0/link/lidar_link/sensor/lidar_2d_v2/scan:=/scan & '
                 'ros2 run ros_gz_bridge parameter_bridge '
                 '/pi_camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image '
                 '/pi_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo '
                 '/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image '
                 '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo; exec bash'],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen'
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_path, 'rviz', 'display.rviz')]
        )
    ])
