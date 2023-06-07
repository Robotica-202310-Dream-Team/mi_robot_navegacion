#!/usr/bin/env python3 
import rclpy
import time
import serial
import json
from rclpy.node import Node
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

        self.pwl = 0
        self.pwr = 0
        self.msgr = Bool()
        self.msgs = Bool()
        self.msg1 = Float32MultiArray()
        self.time_analisis = 7
        self.param = float(input("pwm: "))
        
        super().__init__('mi_robot_naveacion_manipulacion')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                        history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                        depth=1)
    
        self.publisher_recoger = self.create_publisher(Bool, 'pick_up', 50)
        self.publisher_soltar = self.create_publisher(Bool, 'release', 50)
        self.subscriber_banner = self.create_subscription(Bool, 'panel_flag', self.callback_flag, 50)
        self.publisher_vel = self.create_publisher(Float32MultiArray, 'robot_cmdVel', 50)
        
    def callback_flag(self, msg):
        self.ruta = msg.data
        self.mover()

    def mover(self):
        if self.ruta == True:
            # Panel 2
            self.forward(16)
            time.sleep(3)
            self.girol(16)
            time.sleep(3)
            #self.giror(35)

            # Recoger
            #time.sleep  
            self.msgr.data = True
            self.publisher_recoger.publish(self.msgr)
            time.sleep(self.time_analisis)
            self.msgr.data = False
            self.publisher_recoger.publish(self.msgr)
            self.giror(17)
            time.sleep(3)
            self.forward(1)
            time.sleep(3)
            #self.giror(20)
            #time.sleep(3)
            #self.forward(1)
            #time.sleep(3)
            #self.giror(18)
            #time.sleep(3)
            #self.forward(1)
            #time.sleep(3)

            # Soltar
            time.sleep(1) 
            self.msgs.data = True
            self.publisher_soltar.publish(self.msgs)
            time.sleep(self.time_analisis)
            self.msgs.data = False
            self.publisher_soltar.publish(self.msgs)
            #self.giror(40)
            #time.sleep(3)
            #self.forward(60)
            #time.sleep(3)
            #self.giror(15)
            #time.sleep(3)
            print("Termino recorrido 1")
    
        elif self.ruta == False:
            # Panel 2
            self.forward(100)
            time.sleep(3)
            self.giror(50)
            time.sleep(3)
            self.backward(13)
            time.sleep(3)

            # Recoger
            
            self.msgr.data = True
            self.publisher_recoger.publish(self.msgr)
            self.msgs.data = False
            self.publisher_soltar.publish(self.msgs)
            time.sleep(self.time_analisis)
            time.sleep(self.time_analisis)
            self.giror(40)
            time.sleep(3)
            self.forward(100)
            time.sleep(3)
            self.giror(20)
            time.sleep(3)

            # Soltar
            self.msgr.data = False
            self.publisher_recoger.publish(self.msgr)
            self.msgs.data = True
            self.publisher_soltar.publish(self.msgs)
            '''
            self.forward(20)
            time.sleep(3)
            self.giror(6)
            time.sleep(3)
            self.forward(20)
            time.sleep(3)
            self.msgr.data = False
            self.publisher_recoger.publish(self.msgr)
            self.msgs.data = True
            self.publisher_recoger.publish(self.msgs)
            time.sleep(self.time_analisis)
            '''
            print("Termino recorrido1 ")

    def forward(self, n):
        
        i = 0
        while i < n:
            self.pwl = self.param
            self.pwr = self.param
            self.msg1.data = [float(self.pwl),float(self.pwr) ]
            self.publisher_vel.publish(self.msg1)
            time.sleep(0.05)
            print(i)
            i+= 1


    def backward(self, n):
        i = 0
        while i < n:
            self.pwl = -self.param
            self.pwr = -self.param
            self.msg1.data = [float(self.pwl),float(self.pwr) ]
            self.publisher_vel.publish(self.msg1)
            time.sleep(0.05)
            print(i)
            i+= 1
    def giror(self, n):

        self.msg1.data = [float(0),float(0) ]
        self.publisher_vel.publish(self.msg1)
        i = 0
        while i < n:
            self.pwl = self.param
            self.pwr = -self.param
            self.msg1.data = [float(self.pwl),float(self.pwr) ]
            self.publisher_vel.publish(self.msg1)
            time.sleep(0.05)
            print(i)
            i+= 1
        
        self.msg1.data = [float(0),float(0) ]
        self.publisher_vel.publish(self.msg1)
    def girol(self, n):

        self.msg1.data = [float(0),float(0) ]
        self.publisher_vel.publish(self.msg1)
        i = 0
        while i < n:
            self.pwl = -self.param
            self.pwr = self.param
            self.msg1.data = [float(self.pwl),float(self.pwr) ]
            self.publisher_vel.publish(self.msg1)
            time.sleep(0.05)
            print(i)
            i+= 1
        
        self.msg1.data = [float(0),float(0) ]
        self.publisher_vel.publish(self.msg1)

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
