import rclpy
from smach import State
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Bool
from px4_msgs.msg import VehicleLocalPosition, VehicleCommand

class ExitOk(Exception): pass
class NodeState(State):
    def __init__(self):
        State.__init__(self, outcomes=["succeeded", "aborted"])
    def execute(self, userdata):
        try:
            node = LandNode()
            rclpy.spin(node)
        except ExitOk:
            node.destroy_node()
            return "succeeded"
        except Exception as e:
            print(e)
            return "aborted"
class LandNode(Node):
    def __init__(self):
        super().__init__("landing_node")
        self.get_logger().info("Starting landing process...")

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.max_vel_z = -0.15

        self.z_ref = -0.10
        self.z_current = 0.0

        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.vehicle_local_position_sub = self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self.vehicle_local_position_update, qos_profile)
        self.do_height_control_pub = self.create_publisher(Bool, "/px4_driver/do_height_control", 10)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos_profile)

        self.declare_parameter("do_height_control", False)

        self.do_height_control = self.get_parameter("do_height_control").get_parameter_value().bool_value
        self.ts = 0.064

        self.heartbeat_timer = self.create_timer(self.ts, self.control)

    def control(self):
        
        do_height_control_msg = Bool()
        do_height_control_msg.data = self.do_height_control
        self.do_height_control_pub.publish(do_height_control_msg)

        if self.z_current < self.z_ref:
            msg = Twist()
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = self.max_vel_z
            self.vel_pub.publish(msg) 
        else:
            self.disarm()
            raise ExitOk()
    
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

def main(args=None):
    rclpy.init(args=args)
    node = LandNode()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(e)