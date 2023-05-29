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

        # PID constants
        self.K_rho = 0.15 
        self.K_alpha = 0.5 
        self.K_beta = 0.0

        # Geometrical conditions
        self.wheel_radius = 6.5/100
        self.wheel_separation = 19.5/100

        # Variables for monitor the position error
        self.errorPos_x = []
        self.errorPos_y = [] 
        self.errorTheta = []

        # Variables for monitor the hostorical position of the robot
        self.historicalPose_x = []
        self.historicalPose_y = [] 
        self.historicalPose_Theta = []

        super().__init__('mi_robot_pose')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                        history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                        depth=1)
        
        self.sub_pos_final = self.create_subscription(Float32MultiArray, 'posicion_final' ,self.subscriber_callback_pos_final, 10)
        self.sub_pos_actual = self.create_subscription(Odometry, 'camera/pose/sample' ,self.subscriber_callback_pos_actual, qos_profile=qos_policy)
        self.publisher_ = self.create_publisher(Bool, 'llego', 10)

        #self.subscription = self.create_subscription(Odometry, 'camera/pose/sample', self.listener_callback, 10)
    
    def subscriber_callback_pos_actual(self, msg):
        # Alineation flag
        self.align_flag = False

        # Actual position of the robot
        self.actual_pos_x = round (msg.pose.pose.position.x*100,2 )
        self.actual_pos_y = round (msg.pose.pose.position.y *100,2 )
        self.actual_pos_Theta = round (msg.pose.pose.orientation.z *100,2 )

        # Historical position append
        self.historicalPose_x.append(self.actual_pos_x)
        self.historicalPose_y.append(self.actual_pos_y)
        self.historicalPose_Theta.append(self.actual_pos_Theta)
        self.position_error_new()
        
    
    def subscriber_callback_pos_final(self, msg):
        # Position goal
        self.final_pose_x = msg.data[0]
        self.final_pose_y = msg.data[1]
        self.final_pose_Theta = msg.data[2]

    
    def position_error_new(self):
        # Error calculation and append
        self.errorPos_x.append(self.final_pose_x - self.actual_pos_x[-1])
        self.errorPos_y.append(self.final_pose_y - self.actual_pos_y[-1])
        self.errorTheta.append(self.final_pose_Theta - self.actual_pos_Theta[-1])
        print(self.errorPos_x, self.errorPos_y, self.errorTheta)


def main(args=None):
    rclpy.init(args=args)
    mi_robot_pose=navegacion()
    rclpy.spin(mi_robot_pose)
    mi_robot_pose.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
