#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleLocalPosition, VehicleCommand
from geometry_msgs.msg import Twist
import time
from smach import State

class ExitOk(Exception):
    pass

class NodeState(State):
    def __init__(self):
        super().__init__(outcomes=['succeeded', 'aborted'])

    def execute(self, userdata):
        try:
            node = LandNode()
            rclpy.spin(node)
        except ExitOk:
            # detener nodo y salir bien
            node.destroy_node()
            # rclpy.shutdown()
            return 'succeeded'
        except Exception as e:
            print("Error en estado Land:", e)
            return 'aborted'

class LandNode(Node):
    def __init__(self):
        super().__init__('land_node')
        self.get_logger().info('land_node node started')


        self.cmd_pub = self.create_publisher(Twist, '/px4_driver/cmd_vel', 10)
        self.do_height_control_pub = self.create_publisher(Bool, "/px4_driver/do_height_control", 10)

        self.timer = self.create_timer(self.ts, self.control_loop)
        self.last_time = time.time()

    def control_loop(self):
        current_time = time.time()
        msg = Twist()

        
        
    
    def vehicle_local_position_update(self, msg):
        self.vehicle_local_position = msg
        self.z_current = msg.z
    
    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

    def disarm(self):
        self.get_logger().info('Disarming vehicle')
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0, param2=21196.0)

    def stop_drone(self):
        twist = Twist()
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = CrossTunnelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(e)
