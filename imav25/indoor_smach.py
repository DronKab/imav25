import rclpy
from rclpy.node import Node
import smach
import smach_ros
import time 
from imav25 import start_msg, goto_zone
from std_msgs.msg import Empty, Float32

class IndoorSmach(Node):
    def __init__(self):
        super().__init__("indoor_smach")
        self.get_logger().info("State Machine node started")

        self.takeoff_pub = self.create_publisher(Empty, "/px4_driver/takeoff", 10)
        self.change_height_pub = self.create_publisher(Float32, "/px4_driver/target_height", 10)

        # Create state machine
        sq = smach.Sequence(outcomes=["succeeded", "aborted", "preempted"], connector_outcome="succeeded")

        with sq:
            # smach.Sequence.add("WAIT_FOR_START_MSG", start_msg.NodeState())
            # smach.Sequence.add("INITIAL TAKEOFF", smach.CBState(self.takeoff, outcomes=["succeeded"]))
            smach.Sequence.add("HEIGHT_takeoff", smach.CBState(self.control_height, input_keys=["altura"], cb_args=[1.0], outcomes=["succeeded"]))
            smach.Sequence.add("GOTO_TUNNELS", goto_zone.NodeState(1.0, 1.0, 90))

        # Start server for state machine visualization
        server = smach_ros.IntrospectionServer('indoor_smach_server', sq, '/SM_ROOT')
        server.start()

        # Execute state machine
        outcome = sq.execute()
        self.get_logger().info(f"State Machine ended with outcome {outcome}")

        
    def takeoff(self, userdata):
        self.get_logger().info("Publishing takeoff msg")
        self.takeoff_pub.publish(Empty())
        return "succeeded"

    def control_height(self, userdata, altura):
        msg = Float32()
        msg.data = altura
        self.get_logger().info(f"Changed height target to {msg.data}")
        self.change_height_pub.publish(msg)
        return "succeeded"
    
    def delay(self, userdata, secs):
        seconds = secs
        time.sleep(seconds)
        return "succeeded"

def main(args=None):

    rclpy.init(args=args)
    indoor_smach = IndoorSmach()
    rclpy.spin(indoor_smach)

    indoor_smach.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()