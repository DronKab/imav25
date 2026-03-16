#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import math
import time
from smach import State

class ExitOk(Exception):
    pass

class NodeState(State):
    def __init__(self, x=0.0, y=0.0, yaw=90):
        State.__init__(self, outcomes=["succeeded", "aborted"])
        self.x = x
        self.y = y
        self.yaw = yaw

    def execute(self, userdata):
        node = None
        try:
            #rclpy.init()
            node = GotoZone(self.x, self.y, self.yaw)
            node.run()
            rclpy.spin(node)
        except ExitOk:
            if node:
                node.destroy_node()
            # rclpy.shutdown()
            return "succeeded"
        except Exception as e:
            print(f"[GOTO_ZONE ERROR]: {e}")
            if node:
                node.destroy_node()
            # rclpy.shutdown()
            return "aborted"


class GotoZone(Node):
    def __init__(self, x_tar=0.0, y_tar=0.0, yaw_tar=90):
        super().__init__('goto_zone')
        self.x_tar = x_tar
        self.y_tar = y_tar
        self.yaw_tar = yaw_tar
        self.navigator = BasicNavigator()

    def run(self):
        self.get_logger().info("Esperando Nav2...")
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2 activo — enviando objetivo")
        self.go_to_goal(self.x_tar, self.y_tar, self.yaw_tar)

    def go_to_goal(self, x, y, yaw_deg):
        yaw = math.radians(yaw_deg)

        q = Quaternion()
        q.w = math.cos(yaw / 2.0)
        q.z = math.sin(yaw / 2.0)

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        goal.pose.orientation = q

        self.navigator.goToPose(goal)

        while not self.navigator.isTaskComplete():
            fb = self.navigator.getFeedback()
            if fb and fb.distance_remaining is not None:
                self.get_logger().info(f"Distancia restante: {fb.distance_remaining:.2f} m")
            time.sleep(0.5)

        result = self.navigator.getResult()

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Objetivo alcanzado")
            raise ExitOk

        elif result == TaskResult.CANCELED:
            self.get_logger().info("Trayectoria cancelada")
        else:
            self.get_logger().info("Fallo al alcanzar objetivo")
            raise Exception("Falló la navegación")

def main(args=None):
    rclpy.init(args=args)
    try:
        node = GotoZone(0.5, -0.5, 90)  # Valores de prueba
        node.run()
        rclpy.spin(node)
    except ExitOk:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(e)
