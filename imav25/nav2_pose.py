#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav2_simple_commander.robot_navigator import BasicNavigator
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class Nav2PoseNode(Node):
    def __init__(self):
        super().__init__("nav2_pose")
        self.get_logger().info('nav2_pose node started')

        self.navigator = BasicNavigator()

        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)

        # Publicador de pose del dron
        self.pose_pub = self.create_publisher(PoseStamped, "/drone_pose", 10)

        self.current_pose = None

    def odom_callback(self, msg: Odometry):
        # Guardar la pose
        self.current_pose = PoseStamped()
        self.current_pose.header = msg.header
        self.current_pose.pose = msg.pose.pose

        # Publicar la pose
        self.pose_pub.publish(self.current_pose)

        self.get_logger().info(f"Pose: x={self.current_pose.pose.position.x:.2f}, " f"y={self.current_pose.pose.position.y:.2f}, "f"z={self.current_pose.pose.position.z:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = Nav2PoseNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.navigator.lifecycleShutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
