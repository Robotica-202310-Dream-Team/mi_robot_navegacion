import cv2
import os 
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import turtle
import tkinter as tk
from tkinter import filedialog
from pathlib import Path
import sys
#from mi_robot_srv_12.srv import ReadTxt
import threading
import time
from nav_msgs.msg import Odometry
from threading import Thread

# Este codigo se suscribe al topico con mensaje tipo Twist de posicon
# y grafica en tiempo real la posición del robot en un topico con 
# mensaje tipo Image
global height, width
global poses_new, mapa, mapa_actual

#Posicion instantanea del robot (x,y)

#Variables para acumular todas las posiciones
mapa_actual = 255*np.ones((500,500),dtype=np.uint8) # imagen en blanco
mapa = mapa_actual 
height=500
width=500
########################################################################################################################

class Robot_interface(Node):
    global height, width
    def __init__(self):
        super().__init__('mi_robot_interfaz')
        self.pos_x = 0
        self.pos_y = 0
        self.pos_x_total = [0]
        self.pos_y_total = [0]
        self.poses_new  = [0,0]
        #self.publisher_ = self.create_publisher(String, 'turtle_bot_image', 10) Publica un string
        #self.publisher_ = self.create_publisher(Image, 'robot_image', 10) #publica una imagen
        #self.cli = self.create_client(ReadTxt, 'read_txt')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                        history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                        depth=1)
        self.sub = self.create_subscription(Odometry, 'camera/pose/sample' ,self.subscriber_callback, qos_profile=qos_policy)
        self.infoGuardar = self.create_publisher(String, 'info_guardar_txt', 10)

        self.br = CvBridge()
        self.mapa_base =  255*np.ones((500,500),dtype=np.uint8)
        #self.req = ReadTxt.Request()
        root = tk.Tk()
        root.title("Robot_interface Parlington")
        self.canvas = tk.Canvas(master = root,highlightbackground="#351f54", width = 600, height = 600)
        self.t = turtle.RawTurtle(self.canvas)
        self.canvas.pack()
        self.t.pencolor("black")
        self.nick = tk.StringVar()
        tk.Button(master = root, background="#c35bcf" , text = "Teleoperar",font="helvetica 10", command = self.boton1).pack(side = tk.LEFT)
        tk.Button(master = root, background="#c35bcf", text = "Screenshoot",font="helvetica 10", command=self.boton2).pack(side=tk.LEFT)
        tk.Button(master = root, background="#c35bcf", text = "Replicar recorrido",font="helvetica 10", command=self.boton3).pack(side = tk.LEFT)
        tk.Button(master = root, background="#c35bcf", text = "Guardar teleop",font="helvetica 10", command=self.boton4).pack(side = tk.LEFT)
        #tk.Button(master = root, background="#c35bcf", text = "Funcioanlidad 5",font="helvetica 10", command=self.boton5).pack(side = tk.LEFT)
        tk.Label(root,background="#c35bcf",  text="File name:",font="helvetica 10").pack(side = tk.LEFT,)
        self.insert_nick = tk.Entry(root, background="#a5e1f2", width=20,  textvariable=self.nick).pack(side = tk.LEFT)
        
        #tk.Label(master = root, text = "Label").pack(side = tk.LEFT)
        #minimal_subscriber_publisher = MinimalPublisher_suscriber()
        print ("minimal_subscriber_publisher")
        #rclpy.spin(minimal_subscriber_publisher)
        #minimal_subscriber_publisher.destroy_node()
        #rclpy.shutdown()
        root.mainloop()
        
        #################################################################################################################################################33
    def send_request(self, txt):
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req.mensaje = txt
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def subscriber_callback(self, msg):
        self.pos_x = round (msg.pose.pose.position.x*100,2 )
        self.pos_y = round (msg.pose.pose.position.y *100,2 )
        self.pos_z = round (msg.pose.pose.position.z *100,2 )
        
        print (f"x = {self.pos_x} y = {self.pos_y} z = {self.pos_y}")


        #print ("not shutdown")
        self.poses_new = pixels((self.pos_x,self.pos_y),height,width)

        self.pos_x_total.append(self.poses_new[0])
        self.pos_y_total.append(self.poses_new[1])
        
        #for i in range(len(pos_y_total)):
        
        print (f"x = {self.pos_x_total[-1]} y = {self.pos_y_total[-1]}, i= {len(self.pos_x_total)}")
        self.t.goto (self.poses_new[0], self.poses_new[1])
        
                
    
    
    def boton1(self):
        #t.forward(1)
        print ("boton1")
        thread = threading.Thread(target=rclpy.spin(self))
        thread.start()
        self.t.pencolor("red")
        self.t.pensize(2)
        
        
        
    
    def boton2 (self):
        print ("Boton2")
        scriptDir = filedialog.askdirectory()
        name =self.nick.get()
        ruta = scriptDir + "/"+name
        print (ruta)
        self.canvas.postscript(file=ruta, colormode='color')
        pass
        
    def boton3(self):
        print ("Boton3")
        scriptDir = filedialog.askdirectory()
        name =self.nick.get()
        ruta = scriptDir + "/"+name
        self.req.mensaje = ruta
        response = self.send_request(str(self.req.mensaje))
        print ("respuesta?????")
        self.get_logger().info('Result reading txt: '+ response.respuesta)
        pass

    def boton4(self):
        print ("Guardar Teleop")
        scriptDir = filedialog.askdirectory()
        res =self.nick.get()
        lista = res.split(",")
        respSiNo = lista[0]
        name = lista[1]
        ruta = scriptDir + "/" + name
        msg = String()
        msg.data = (str(respSiNo) + "," + ruta)
        self.infoGuardar.publish(msg)





##############################################################################################################################

def pixels (coord,height, width): # Pasar de metros a pixeles 
    x_len = 100 # metros
    y_len = 100 # metros
    #x_center = width//2
    #y_center = height//2
    print (width)
    x =  round((coord[0]*width // x_len ) ) 
    y =  round((coord [1]*height// y_len) )   
    return [int(x),int(y)]
    
def main(args=None):    
    rclpy.init(args=args)
    robot_interface = Robot_interface()
    print("robot_interface")
    
    Robot_interface.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
