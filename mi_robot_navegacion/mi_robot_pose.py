#!/usr/bin/env python3 
import rclpy
import time
import serial
import json
from rclpy.node import Node
#from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32MultiArray, Bool

# Este codigo sirve para la comunicacion serial entre un Arduino/ESP y una Raspberry. Es un nodo de ros2 que se subscribe 
# al tópico cmd_Vel y lee el mensaje para pasarlo luego al Arduino/ESP. Los mensajes son tipo String. 

# Este codigo es para ROS2.

# Basado en el trabajo hecho por  el subsistema motion control ROBOCOl Colombia,
# Implementación por Robocol en ROS1:https://github.com/Motion-control-rem-u/motion_control_rem_u.git 

class navegacion(Node):

    def __init__(self):
        print ("Incia el nodo")
        super().__init__('mi_robot_pose')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                        history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                        depth=1)
        self.sub_pos_actual = self.create_subscription(Odometry, 'camera/pose/sample' ,self.subscriber_callback_pos_actual, qos_profile=qos_policy)
        self.sub_pos_final = self.create_subscription(Float32MultiArray, 'posicion_final' ,self.subscriber_callback_pos_final, 10)
        self.publisher_ = self.create_publisher(Bool, 'llego', 10)

        #self.subscription = self.create_subscription(Odometry, 'camera/pose/sample', self.listener_callback, 10)
        

    def subscriber_callback_pos_actual(self, msg):
        #self.pos_x = round (msg.twist.twist.linear.x / 100, 2)
        #self.pos_y = round (msg.twist.twist.linear.y / 100, 2)
        #self.pos_z = round (msg.twist.twist.linear.z / 100, 2)
        #debug = msg.pose.pose.position.
        #print (f"debug: {debug}")
        self.pos_x = round (msg.pose.pose.position.x*100,2 )
        self.pos_y = round (msg.pose.pose.position.y *100,2 )
        self.pos_z = round (msg.pose.pose.position.z *100,2 )
        print (f"x = {self.pos_x} y = {self.pos_y} z = {self.pos_y}")
    

def main(args=None):
    rclpy.init(args=args)
    mi_robot_pose=navegacion()
    rclpy.spin(mi_robot_pose)
    mi_robot_pose.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
