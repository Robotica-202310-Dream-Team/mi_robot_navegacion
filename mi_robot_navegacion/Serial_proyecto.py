#!/usr/bin/env python3 
import rclpy
import time
import serial
import json
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32MultiArray

# Este codigo sirve para la comunicacion serial entre un Arduino/ESP y una Raspberry. Es un nodo de ros2 que se subscribe 
# al tópico cmd_Vel y lee el mensaje para pasarlo luego al Arduino/ESP. Los mensajes son tipo String. 

# Este codigo es para ROS2.

# Basado en el trabajo hecho por  el subsistema motion control ROBOCOl Colombia,
# Implementación por Robocol en ROS1:https://github.com/Motion-control-rem-u/motion_control_rem_u.git 

class serialRaspESP(Node):

    def __init__(self):

        super().__init__('Serial_proyecto')
        self.subscription = self.create_subscription(Float32MultiArray, 'robot_cmdVel', self.listener_callback_velocidad, 50)    
        print("Inicio del nodo que pasa la informacion de la Raspberry al Arduino")
        try:
            ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            ser.reset_input_buffer()   
            print("Conexion Serial exitosa")    
        except:
            print("Conexion Fallida")

    def listener_callback_velocidad(self, msg):
        left = agregar_ceros(int(msg.data[0]))
        right = agregar_ceros(int(msg.data[1]))
        #print (f"left{left}")
        pwms = [left,right]          
        pwm = (str(pwms)  + "\n").encode('utf-8')
        print (pwm)
        serial.Serial('/dev/ttyUSB0', 115200, timeout=1).write(pwm)



def agregar_ceros(numero):
        #print (f"numero{numero}")
        es_positivo=numero>=0
        numero_str=str(abs(numero))
        if es_positivo:
            numero_str="0"*(4-len(numero_str))+numero_str
        else:
            numero_str="-"+"0"*(3-len(numero_str))+numero_str
        return numero_str


def main(args=None):
    rclpy.init(args=args)
    SerialRaspESP=serialRaspESP()
    rclpy.spin(SerialRaspESP)
    SerialRaspESP.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
