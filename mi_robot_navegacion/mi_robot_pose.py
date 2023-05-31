#!/usr/bin/env python3 
import rclpy
import time
import serial
import json
from rclpy.node import Node
#from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32MultiArray, Bool
import numpy as np
import math
# Este codigo sirve para la comunicacion serial entre un Arduino/ESP y una Raspberry. Es un nodo de ros2 que se subscribe 
# al tópico cmd_Vel y lee el mensaje para pasarlo luego al Arduino/ESP. Los mensajes son tipo String. 

# Este codigo es para ROS2.

# Basado en el trabajo hecho por  el subsistema motion control ROBOCOl Colombia,
# Implementación por Robocol en ROS1:https://github.com/Motion-control-rem-u/motion_control_rem_u.git 

class navegacion(Node):

    def __init__(self):
        print ("Inicia el nodo que da la posicion de la camara en tiempo real")
        self.max = 1
        # PID constants
        self.err_ang = 28
        self.err_dist = 5
        self.K_rho = 0.15 
        self.K_alpha = 0.5 
        self.K_beta = 0.0

        self.banderaOrientacion = False
        # Geometrical conditions
        self.wheel_radius = 6.5/100
        self.wheel_separation = 19.5/100

        # Variables for monitor the position error
        self.errorPos_x = []
        self.errorPos_y = [] 
        self.errorTheta = [0]
        self.llegoPosFinal = False

        # Variables for monitor the hostorical position of the robot
        self.historicalPose_x = []
        self.historicalPose_y = [] 
        self.historicalPose_Theta = []
        self.final_pose_x = 1
        self.final_pose_y = 1
        self.final_pose_Theta = 1
        self.actualGrado = 0
        super().__init__('mi_robot_pose')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                        history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                        depth=1)
        
        self.sub_pos_final = self.create_subscription(Float32MultiArray, 'posicion_final' ,self.subscriber_callback_pos_final, 50)
        self.sub_pos_actual = self.create_subscription(Odometry, 'camera/pose/sample' ,self.subscriber_callback_pos_actual, qos_profile=qos_policy)
        self.publisher_ = self.create_publisher(Bool, 'llego', 50)
        self.publisher_vel = self.create_publisher(Float32MultiArray, 'robot_cmdVel', 50)
        self.msg1 = Float32MultiArray()
        self.control_variables(0)

        #self.subscription = self.create_subscription(Odometry, 'camera/pose/sample', self.listener_callback, 10)
    
    def subscriber_callback_pos_actual(self, msg):
        # Actual position of the robot
        self.actual_pos_x = round (msg.pose.pose.position.x*100,2 )
        self.actual_pos_y = round (msg.pose.pose.position.y *100,2 )
        self.actual_pos_ThetaX = round (msg.pose.pose.orientation.x,2 )
        self.actual_pos_ThetaY = round (msg.pose.pose.orientation.y,2 )
        self.actual_pos_ThetaZ = round (msg.pose.pose.orientation.z,2 )
        self.actual_pos_ThetaW = round (msg.pose.pose.orientation.w,2 )

        self.actualGrado2 = self.euler_from_quaternion(self.actual_pos_ThetaX,self.actual_pos_ThetaY,self.actual_pos_ThetaZ,self.actual_pos_ThetaW)
        self.actualGrado = self.actualGrado2[2]*180/(np.pi)
        print("x_actual: "+ str(self.actual_pos_x) +"\n")
        print("y_actual: "+ str(self.actual_pos_y)+"\n") 
        print("ThetaZ_actual: "+ str(self.actualGrado)+"\n")
        
        # Historical position append
        self.historicalPose_x.append(self.actual_pos_x)
        self.historicalPose_y.append(self.actual_pos_y)
        self.historicalPose_Theta.append(self.actualGrado)

        # Historical error calculation
        if self.llegoPosFinal == True:
            self.position_error_new()
            print(f"Alpha =  {self.alpha}  rho= {self.rho}" )
            
            # Orientation control
            if  abs(self.alpha) > self.err_ang:
                self.position_error_new() 
                self.banderaOrientacion = False
                self.control_variables(1)
                self.orientation_goal()
                
                if abs(self.alpha) <= self.err_ang:
                    print("Ya se oriento")
                    self.PWMR = self.PWML
                    self.PWML = -self.PWMR
                    self.msg1.data = [float(self.PWML), float(self.PWMR)]
                    self.publisher_vel.publish(self.msg1)
                    time.sleep(5)
                    self.banderaOrientacion = True
                self.msg1.data = [float(self.PWML), float(self.PWMR)]
                self.publisher_vel.publish(self.msg1)
                
            # Linear control
            if self.rho > self.err_dist and self.banderaOrientacion == True:
                print("Entro al control de distancia")
                self.position_error_new()
                self.control_variables(2)
                self.linear_trayectory()
                self.msg1.data = [self.PWML, self.PWMR]
                self.publisher_vel.publish(self.msg1)
                
                if abs(self.alpha) > self.err_ang:
                    self.banderaOrientacion = False
                else:
                    self.position_error_new()
                    self.control_variables(2)
                    self.linear_trayectory()
                    self.msg1.data = [self.PWML, self.PWMR]
                    self.publisher_vel.publish(self.msg1)

            elif self.rho <= self.err_dist:
                self.banderaOrientacion = True
                self.PWML = 0.0
                self.PWMR = 0.0
                self.msg1.data = [float(self.PWML), float(self.PWMR)]
                self.publisher_vel.publish(self.msg1)
                print("El robot ha llegado al destino")

    def control_variables(self, instance):
        # Control variables actualization
        if instance == 0:
            self.rho = round (np.sqrt(self.final_pose_x**2 + self.final_pose_y**2),2)
            self.alpha = round (-1*self.actualGrado + (np.arctan2(self.final_pose_y, self.final_pose_x))*180/np.pi,2)
            self.beta = self.errorTheta[-1]

        elif instance == 1:
            self.rho = round(np.sqrt(self.errorPos_x[-1]**2 + self.errorPos_y[-1]**2),2)
            self.beta = self.errorTheta[-1]

        elif instance == 2:
            self.rho =round( np.sqrt(self.errorPos_x[-1]**2 + self.errorPos_y[-1]**2),2)
            self.alpha = round(-1*self.historicalPose_Theta[-1] + (np.arctan2(self.errorPos_y[-1], self.errorPos_x[-1]))*180/np.pi , 2)
            self.beta = self.errorTheta[-1]

    def linear_trayectory(self):
        # Angular veocity definition and limit set
        self.PWMR = float(60)
        self.PWML = float(60)
    
    def orientation_goal(self):
        # Angle diff calculation
        self.delta_Theta = (np.arctan2(self.errorPos_y[-1], self.errorPos_x[-1]))*180/np.pi

        # Variable control alpha update
        self.alpha = round( self.delta_Theta - self.historicalPose_Theta[-1],2)
        if self.alpha < -180.0:
            set.alpha += 360.0
        if self.alpha > 180.0:
            set.alpha -= 360.0

        if self.delta_Theta >= 0:
            # Linear velocities calculation: CCW
            self.PWMR = float(60)
            self.PWML = float(-60)
        if self.delta_Theta < 0:
            # Linear velocities calculation: CW
            self.PWMR = float(-60)
            self.PWML = float(60)
    
    def subscriber_callback_pos_final(self, msg):
        # Position goal
        self.final_pose_x = msg.data[0]/4
        self.final_pose_y = msg.data[1]/4
        self.final_pose_Theta = 0
        self.llegoPosFinal = True

    def position_error_new(self):
        # Error calculation and append
        self.errorPos_x.append(self.final_pose_x - self.actual_pos_x)
        self.errorPos_y.append(self.final_pose_y - self.actual_pos_y)
        self.errorTheta.append(self.final_pose_Theta - self.actualGrado)
        print("x_error: "+ str(self.errorPos_x[-1]) +"\n")
        print("y_error: "+ str(self.errorPos_y[-1])+"\n") 

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return float(roll_x), float(pitch_y), float(yaw_z) # in radians
    
def main(args=None):
    rclpy.init(args=args)
    mi_robot_pose=navegacion()
    rclpy.spin(mi_robot_pose)
    mi_robot_pose.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
