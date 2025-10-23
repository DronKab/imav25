#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import Empty
import pygame
import threading
import time


class PygameMoveDrone(Node):
    def __init__(self):
        super().__init__('keyboard_move_drone')
        self.get_logger().info('Keyboard control node (pygame) started')

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.height_pub = self.create_publisher(Bool, '/px4_driver/do_height_control', 10)

        self.arm_pub = self.create_publisher(Empty, '/px4_driver/takeoff', 10)

        # Parameters
        self.declare_parameter('scale_linear', 2.0)
        self.declare_parameter('scale_angular', 2.0)
        self.declare_parameter('do_height_control', False)

        self.do_height_control = self.get_parameter('do_height_control').value
        self.running = True
        self.twist = Twist()

        # Start pygame in another thread (non-blocking)
        self.key_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.key_thread.start()

        # Publish rate
        self.timer = self.create_timer(0.1, self.publish_cmd)

    def keyboard_loop(self):
        pygame.init()
        pygame.display.set_mode((200, 100))
        pygame.display.set_caption("Drone Keyboard Control (Press ESC to quit)")

        linear_speed = self.get_parameter('scale_linear').value
        angular_speed = self.get_parameter('scale_angular').value

        self.get_logger().info(
            "Controls:\n"
            "W/S: forward/back\n"
            "A/D: left/right\n"
            "↑/↓: ascend/descend\n"
            "Q/E: yaw left/right\n"
            "ESC: quit"
        )

        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    self.running = False

             

            # Reset twist
            twist = Twist()
            keys = pygame.key.get_pressed()

            if keys == pygame.K_0:  # ASCII value for space
                #arm_toggle = not arm_toggle  # Flip the value of arm_toggle
                arm_msg = Empty()  # Nota los paréntesis
                self.arm_pub.publish(arm_msg)

            # --- Linear X (forward/back) ---
            if keys[pygame.K_w]:
                twist.linear.x = linear_speed
            elif keys[pygame.K_s]:
                twist.linear.x = -linear_speed

            # --- Linear Y (left/right) ---
            if keys[pygame.K_a]:
                twist.linear.y = linear_speed
            elif keys[pygame.K_d]:
                twist.linear.y = -linear_speed

            # --- Linear Z (up/down) ---
            if keys[pygame.K_UP]:
                twist.linear.z = linear_speed
            elif keys[pygame.K_DOWN]:
                twist.linear.z = -linear_speed

            # --- Angular Z (yaw) ---
            if keys[pygame.K_q]:
                twist.angular.z = angular_speed
            elif keys[pygame.K_e]:
                twist.angular.z = -angular_speed

            self.twist = twist
            time.sleep(0.05)

        pygame.quit()
        self.get_logger().info("Keyboard control stopped")

    def publish_cmd(self):
        if not self.running:
            rclpy.shutdown()
            return

        # Publish height control state
        msg = Bool()
        msg.data = True
        self.height_pub.publish(msg)

        # Publish twist command
        self.cmd_pub.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    node = PygameMoveDrone()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()