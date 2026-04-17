import rclpy
from rclpy.node import Node
import smach
import smach_ros
import time 
from imav25 import start_msg, goto_zone, ctrl_vision, aruco_control, fly_drone
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
            """
            # Mensaje para comenzar (nice)
            smach.Sequence.add("WAIT_FOR_START_MSG", start_msg.NodeState())
            # el mensaje que se publica es: ros2 topic pub --once /wait_start_msg std_msgs/msg/Empty
            
            # Take off (nice)
            smach.Sequence.add("INITIAL TAKEOFF", smach.CBState(self.takeoff, outcomes=["succeeded"]))

            smach.Sequence.add("DELAY_TAKEOFF", smach.CBState(self.delay, input_keys=["secs"], cb_args=[10], outcomes=["succeeded"]))

            # Altura para tuneles (verificar altura necesaria (o si es necesario ajustar altura)
            smach.Sequence.add("HEIGHT_takeoff", smach.CBState(self.control_height, input_keys=["altura"], cb_args=[1.0], outcomes=["succeeded"]))
            
            # Centrarse en tunel (nice)
            smach.Sequence.add("CTRL_VIS_TUNNEL", ctrl_vision.CtrlVisNodeState(target_class="tunnel", action_flag=True, pos_flag=True))

            # Cruzar tunel (distancia necesaria para cruzar tuneles)
            # smach.Sequence.add("CROSS_TUNNEL", fly_drone.NodeState(x=4.0, y=0.0, yaw=0.0))

            # Acercarse a obstaculos (punto para acercarse a obstaculos)
            smach.Sequence.add("GO_TO_OBSTACLES", fly_drone.NodeState(x=1.0, y=1.0, yaw=1.57079))

            # Evitar obstaculos (nice)
            smach.Sequence.add("AVOID_OBSTACLES", ctrl_vision.CtrlVisNodeState(target_class="obstacle", action_flag=False, pos_flag=True))
            """

            # Centrarse en aruco para pintar tu raya (verificar distancias x,y,z NOTA: las distancias son conforme 
            # al marco de referencia del ARUCO no del DRON)
            smach.Sequence.add("ARUCO_CONTROL", aruco_control.NodeState(x_distance=0.0, y_distance=0.0, z_distance=0.5))
            
            """
            # Moverse para dibujar linea (ajustar distancia y signo en x)
            smach.Sequence.add("DRAW_LINE", fly_drone.NodeState(x=0.0, y=1.5, yaw=0.0))

            # Quitarse del pizarron (verificar el punto x,y y la orientacion)
            smach.Sequence.add("MOVE_FROM_WB", fly_drone.NodeState(x=1.0, y=0.0, yaw=1.57079))

            # Busca plataforma lejana (nice)
            smach.Sequence.add("FIND_FAR_PLATFORM", ctrl_vision.CtrlVisNodeState(target_class="far_plat", action_flag=True, pos_flag=True))

            # Busca plataforma abajo (nice)
            smach.Sequence.add("FIND_PLATFORM", ctrl_vision.CtrlVisNodeState(target_class="top_plat", action_flag=True, pos_flag=False))
            """

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