import rclpy
from std_msgs.msg import Empty
from rclpy.node import Node
from smach import State

class ExitOk(Exception): pass
class NodeState(State):
    def __init__(self):
        State.__init__(self, outcomes=["succeeded", "aborted"])

    def execute(self, userdata):
        try:
            node = NodeStart()
            rclpy.spin(node)
        except ExitOk:
            node.destroy_node()
            return "succeeded"
        except Exception as e:
            print(e)
            return "aborted"

class StartNode(Node):
    def __init__(self):
        super().__init__("start_msg_node")
        self.get_logger().info("Waiting for message to start ...")
        self.aruco_sub = self.create_subscription(Empty, "/wait_start_msg", self.message_callback, 10)

    def message_callback(self, msg):
        self.get_logger().info("Start message received, proceeding with state machine.")
        raise ExitOk 

def main(args=None):
    rclpy.init(args=args)
    node = NodeStart()

    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(e)