#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import smach
import smach_ros
import time
from imav25 import ctrl_vision
from std_msgs.msg import Empty, Float32


class IndoorSmach(Node):
    def __init__(self):
        super().__init__("indoor_smach")
        self.get_logger().info("State Machine node started")
        self.takeoff_pub = self.create_publisher(Empty, "/px4_driver/takeoff", 10)
        self.change_height_pub = self.create_publisher(Float32, "/px4_driver/target_height", 10)

        sq = smach.Sequence(
            outcomes=["succeeded", "aborted", "preempted"],
            connector_outcome="succeeded"
        )
        with sq:
            smach.Sequence.add(
                "CONTROL_TEST",
                ctrl_vision.CtrlVisNodeState(target_class="far_plat", action_flag=True)
            )

        self.sm = sq

        server = smach_ros.IntrospectionServer('indoor_smach_server', sq, '/SM_ROOT')
        server.start()

    def run(self):
        outcome = self.sm.execute()
        self.get_logger().info(f"State Machine ended with outcome: {outcome}")

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
        time.sleep(secs)
        return "succeeded"


def main(args=None):
    rclpy.init(args=args)
    indoor_smach = IndoorSmach()
    indoor_smach.run()        # <-- fuera del __init__, después de rclpy.init()
    indoor_smach.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()