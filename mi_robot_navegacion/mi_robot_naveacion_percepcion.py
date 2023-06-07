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
        self.ruta = 1
        self.pwl = 0
        self.pwr = 0
        self.msg2 = Bool()
        self.msg1 = Float32MultiArray()
        self.time_analisis = 10
        self.param = float(input("pwm: "))
        super().__init__('mi_robot_naveacion_percepcion')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                        history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                        depth=1)
    
        self.publisher_foto = self.create_publisher(Bool, 'tomar_foto', 50)
        self.subscriber_banner = self.create_subscription(Float32MultiArray, 'banners_list', self.callback_banner, 50)
        self.publisher_vel = self.create_publisher(Float32MultiArray, 'robot_cmdVel', 50)
        
    def callback_banner(self, msg):
        self.banners = Float32MultiArray()
        self.banners.data = msg.data
        self.mover()

    def mover(self):
        if self.ruta == 1:
            # Panel 2
            self.forward(30)
            time.sleep(3)
            self.girol(2)
            if self.banners.data[0] == 2.0 or self.banners.data[1] == 2.0:
                self.msg2.data = True
                self.publisher_foto.publish(self.msg2)
                time.sleep(self.time_analisis)
                self.msg2.data = False
                self.publisher_foto.publish(self.msg2)
            else:
                time.sleep(5)
            print("Termino banner 2")

            # Panel 3 
            self.giror(4)
            time.sleep(3)
            self.forward(35)
            time.sleep(3)
            self.giror(10)
            time.sleep(3)
            self.backward(10)
            time.sleep(3)
            self.giror(3)
            time.sleep(3)
            if self.banners.data[0] == 3.0 or self.banners.data[1] == 3.0:
                self.msg2.data = True
                self.publisher_foto.publish(self.msg2)
                time.sleep(self.time_analisis)
                self.msg2.data = False
                self.publisher_foto.publish(self.msg2)
            else:
                time.sleep(5)
            print("Termino banner 3")

            # Panel 1
            self.forward(30)
            time.sleep(3)
            self.giror(15)
            time.sleep(3)
            self.backward(10)
            time.sleep(10)
            if self.banners.data[0] == 1.0 or self.banners.data[1] == 1.0:
                self.msg2.data = True
                self.publisher_foto.publish(self.msg2)
                time.sleep(self.time_analisis)
                self.msg2.data = False
                self.publisher_foto.publish(self.msg2)

            else:
                time.sleep(5)
            self.ruta = 0
            print("Termino banner 1")

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
        i = 0
        while i < n:
            self.pwl = self.param
            self.pwr = -self.param
            self.msg1.data = [float(self.pwl),float(self.pwr) ]
            self.publisher_vel.publish(self.msg1)
            time.sleep(0.05)
            print(i)
            i+= 1
    def girol(self, n):
        i = 0
        while i < n:
            self.pwl = -self.param
            self.pwr = self.param
            self.msg1.data = [float(self.pwl),float(self.pwr) ]
            self.publisher_vel.publish(self.msg1)
            time.sleep(0.05)
            print(i)
            i+= 1

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
