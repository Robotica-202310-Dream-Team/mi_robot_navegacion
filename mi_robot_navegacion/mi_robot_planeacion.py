#!/usr/bin/env python3
import cv2
import time
from heapq import heappop, heappush
import numpy 
import rclpy
import time
import serial
import json
from rclpy.node import Node
#from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32MultiArray, Bool
import numpy as np
import os
from sensor_msgs.msg import Image as Image2

class planeacion(Node):
	def __init__(self):
		print ("Inicia el nodo que realiza la planeación de ruta")
		super().__init__('mi_robot_planeacion')
		qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                        history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                        depth=1)	
		self.sub_pos_final = self.create_subscription(Float32MultiArray, 'posicion_final' ,self.subscriber_callback_pos_final, 50)
		self.sub_pos_actual = self.create_subscription(Odometry, 'camera/pose/sample' ,self.subscriber_callback_pos_actual, qos_profile=qos_policy)
		scriptDir = os.path.dirname(__file__)
		ruta ="/home/sebastian/Uniandes202310/Robotica/proyecto_final/proyecto_final_ws/src/mi_robot_navegacion/mi_robot_navegacion/MapaRobotica.pgm"
		self.x_inicial = 250
		self.y_inicial = 250
		
		#ruta = scriptDir + "/Mapa_Test_drive_3_mod.jpg"
		self.gridmap = cv2.imread(ruta,0) 

		# Taking a matrix of size 5 as the kernel
		kernel = np.ones((90, 90), np.uint8)
		
		# The first parameter is the original image,
		# kernel is the matrix with which image is

		# convolved and third parameter is the number
		# of iterations, which will determine how much
		# you want to erode/dilate a given image.
		self.gridmap = cv2.erode(self.gridmap, kernel, iterations=1)
		ret,self.gridmap = cv2.threshold(self.gridmap,127,255,cv2.THRESH_BINARY)
		self.gridmap = self.gridmap/100
		#cv2.imshow(window_name, self.gridmap)
		#cv2.waitKey(0)  
		#cv2.destroyAllWindows()
		self.height, self.width = self.gridmap.shape

		
		self.PROB_FREE = 0.3
		self.graph = self.gridmap2graph()
		#print (f"graph  : {self.graph}")
		print("* self.gridmap2graph")
	def subscriber_callback_pos_actual(self, msg):
        # Actual position of the robot
		self.x_inicial = round (msg.pose.pose.position.x*100,2 )
		self.y_inicial = round (msg.pose.pose.position.y *100,2 )
		print (f"x inicial= {self.x_inicial} y inicial= {self.y_inicial}")


	def subscriber_callback_pos_final(self, msg):
		self.x_final = msg.data[0]
		self.y_final = msg.data[1]
		print (f"x final= {self.x_final} y final= {self.y_final}")
		
		self.inicio_fin()
		
	def configuration_method(self):
		
		#x_inicial,y_inicial,self.x_final,y_final=es_obstaculo(self.gridmap,x_inicial,y_inicial,self.x_final,y_final,self.width,self.height)
		#declaración coordenadas iniciales y finales
		
		coordenates_array = []
		coordenates_array.append([self.x_inicial,self.y_inicial])

		self.START  =  tuple([self.y_inicial,self.x_inicial])
		self.GOAL   = tuple([self.y_final,self.x_final])

		# types of heuristic function
		heuristica = 2 #seleccion heuristica 

		
		start_time = time.time()

		path = self.Astar( heuristica)
		print("* Astar")
		print("--- %s segundos ---" % (time.time() - start_time))

		pos_actualx = self.x_inicial
		pos_actualy = self.y_inicial

		pos_actualx = self.x_inicial
		pos_actualy = self.y_inicial
		
		for direccion in path:
			if direccion == "N":
				pos_actualx = pos_actualx 
				pos_actualy = pos_actualy -1
				coordenates_array.append([pos_actualx,pos_actualy])

			elif direccion == "W":
				pos_actualx = pos_actualx -1
				pos_actualy = pos_actualy 
				coordenates_array.append([pos_actualx,pos_actualy])
			elif direccion == "S":
				pos_actualx = pos_actualx 
				pos_actualy = pos_actualy + 1
				coordenates_array.append([pos_actualx,pos_actualy])
				
			elif direccion == "E":
				pos_actualx = pos_actualx + 1
				pos_actualy = pos_actualy 
				coordenates_array.append([pos_actualx,pos_actualy])

		
		esquinas =self.coordenates(coordenates_array)
		esquinas = self.depurar_coord (esquinas)
		#self.gridmap = cv2.delate(self.gridmap, self.kernel, iterations=1)
		self.dibujo_ruta2(coordenates_array,esquinas)
		print (f"* {len (esquinas)} Coordenates.")
		
		return esquinas

	def gridmap2graph(self):
		# DO NOT CHANGE
		self.graph = {(i, j): [] for j in range(self.width) for i in range(self.height) if self.gridmap[i][j]>(1-self.PROB_FREE)}
		for row, col in self.graph.keys():
			if row < self.height - 1 and self.gridmap[row + 1][col]>(1-self.PROB_FREE):
				self.graph[(row, col)].append(("S", (row + 1, col)))
				self.graph[(row + 1, col)].append(("N", (row, col)))
			if col < self.width - 1 and self.gridmap[row][col + 1]>(1-self.PROB_FREE):
				self.graph[(row, col)].append(("E", (row, col + 1)))
				self.graph[(row, col + 1)].append(("W", (row, col)))
		return self.graph

	def heuristic(self,cell, type):
		if type == 0:
			#Manhattan distance
			# TODO: write manhattan distance heuristic equation 
			x1, y1 = cell
			x2, y2 = self.GOAL
			return abs(x2-x1) + abs(y2-y1)
			pass
			# return 0
		elif type == 1:
			#Euclidian distance
			x1, y1 = cell
			x2, y2 = self.GOAL
			return numpy.sqrt((x2-x1)**2 + (y2-y1)**2)
			# TODO: write euclidian dimustance heuristic equation 
			pass
		elif type == 2:
			x1,y1 = cell
			x2,y2 = self.GOAL
			mu = 5
			puntos = 30
			penalizacion = 0
			for i in range(mu):
				if not (x1+i,y1) in self.graph:
					penalizacion = penalizacion + puntos
			for i in range(mu):
				if not (x1-i,y1) in self.graph:
					penalizacion = penalizacion + puntos
			for i in range(mu):
				if not (x1,y1+i) in self.graph:
					penalizacion = penalizacion + puntos
			for i in range(mu):
				if not (x1,y1-i) in self.graph:
					penalizacion = penalizacion + puntos
					
			for i in range(mu):
				if not (x1+i,y1+i) in self.graph:
					penalizacion = penalizacion + puntos
			for i in range(mu):
				if not (x1-i,y1-i) in self.graph:
					penalizacion = penalizacion + puntos
			for i in range(mu):
				if not (x1-i,y1+i) in self.graph:
					penalizacion = penalizacion + puntos
			for i in range(mu):
				if not (x1+i,y1-i) in self.graph:
					penalizacion = penalizacion + puntos


			return numpy.sqrt((x2-x1)**2 + (y2-y1)**2) + penalizacion

	def Astar(self, type_h):
		# TODO: 

		encontrado = False
		colaVacia = False

		celda = self.START
		cola = []
		visitados = set()
		print (celda)
		costo = self.gridmap[celda]
		costo_acumulado = 0
		heur = self.heuristic(celda,type_h)
		tupla = (costo+heur,costo_acumulado,celda,"")
		heappush(cola,tupla)

		while encontrado == False and colaVacia == False:

			menor = heappop(cola)
			costo = menor[0]
			costo_acumulado = menor[1]
			celda = menor[2]
			path = menor[3]


			if celda == self.GOAL:
				encontrado  = True
			else:
				if celda not in visitados:

					visitados.add(celda)
					
					for i,j in self.graph[celda]:

						costo2 = self.gridmap[j]
						heur = self.heuristic(j,type_h)

						tupla = (costo_acumulado+heur,costo_acumulado+costo2,j,path+i)

						heappush(cola,tupla)

			if not cola:
				colaVacia = True
		
		if encontrado == True:
			
			return path
		else:
			print("No se encontró un camino directo.")

	def coordenates(self,route):  
		c = []  
		for i in range (2,len (route) -2):     
			if (route[i][0]-route[i-2][0]) !=0 and abs((route[i][1]-route[i-2][1])/(route[i][0]-route[i-2][0])) == 1:
				if (route[i][0]-route[i+2][0]) != 0 and  abs((route[i][1]-route[i+2][1])/(route[i][0]-route[i+2][0])) != 1:
					new_p = route [i]
					c.append(new_p)
			elif route [i][0] == route [i-1][0] :
				if route [i][0] != route [i+1][0] :
					new_p = route [i]
					c.append(new_p)
			elif route [i][1] == route [i-1][1] :
				if route [i][1] != route [i+1][1] :
					new_p = route [i]            
					c.append(new_p)  
			elif route [i][0]-1 == route [i-1][0] and route [i][1]-1 == route [i-1][1]:  
				if route [i][0]+1 != route [i+1][0] or  route [i][1]+1 != route [i+1][1]:
					new_p = route [i]
					c.append(new_p)
		c.append(route [len (route)-1])
		c2=self.menos_puntos(c)
		return  c2

	def menos_puntos (self,esq):
		c2=esq[:]
		menor = 10 #Treshold 
		for i in range(1,len(esq)-1):
			if esq[i-1][0] ==esq[i][0] and esq[i][1] ==esq[i+1][1]:
				if  numpy.sqrt((esq[i-1][0]-esq[i+1][0])**2 + (esq[i-1][1]-esq[i+1][1])**2) <menor:
					c2.remove(esq[i])
			elif esq[i-1][1] ==esq[i][1] and esq[i][0] ==esq[i+1][0]:
				if  numpy.sqrt((esq[i-1][0]-esq[i+1][0])**2 + (esq[i-1][1]-esq[i+1][1])**2) <menor:
					c2.remove(esq[i])
			elif esq[i+1][1] ==esq[i][1] or esq[i+1][0] ==esq[i][0]:
				if  numpy.sqrt((esq[i+1][0]-esq[i][0])**2 + (esq[i+1][1]-esq[i][1])**2) <2:
					c2.remove(esq[i])
		c2 = self.depurar_coord(c2)
		return c2

	def depurar_coord(self,esq):
		c3 = esq[:]
		menor = 10
		for i in range(1,len(esq)-1):
			if  numpy.sqrt((esq[i][0]-esq[i+1][0])**2 + (esq[i][1]-esq[i+1][1])**2) <menor:
				sentinela = numpy.random.randint(1,4)
				if sentinela %2 !=0:
					c3.remove (esq[i])
		return c3


	def dibujo_ruta2(self,array_pos,esquinas):

		pixel = self.gridmap
		matrixMap=numpy.ones([self.height,self.width])
		#print (pixel.shape)
		RGBMap=[]
		for i in range(self.height):
			for j in range(self.width ):
				pixel_actual = pixel[i,j]
				#print (f"width=: {self.width} height: {self.height}")
		#		print (f"valor: {pixel_actual} i: {i} j: {j}")
				if pixel_actual >=2:
					matrixMap[i,j]=1

				else:
					matrixMap[i,j]=0
		for i in range(len(array_pos)):
			matrixMap[array_pos[i][1],array_pos[i][0]]=2
		for i in range(len(esquinas)):
			matrixMap[esquinas[i][1],esquinas[i][0]]=3
		for i in range(self.height):
			file=[]
			for j in range(self.width):
				if matrixMap[i,j]==1:
					RGB=[255,255,255,255] #white
				elif matrixMap[i,j]==0:
					RGB = [0,0,0,255]   #Black
				elif matrixMap[i,j]==2:
					RGB = [255,0,0,255] #Red
				elif matrixMap[i,j]==3: 
					RGB = [0,255,0,255] #Green
				elif matrixMap[i,j]==4: 
					RGB = [0,0,255,255] #Blue
				
				file.append(RGB)
			RGBMap.append(file)
		RGBMap=numpy.array(RGBMap).astype(numpy.uint8)
		
		#routeMap=Image.fromarray(RGBMap,"RGBA")
		ruta ="/home/sebastian/Uniandes202310/Robotica/proyecto_final/proyecto_final_ws/src/mi_robot_navegacion/mi_robot_navegacion/mapafina.png"
		cv2.imwrite (ruta,RGBMap)
		#routeMap.save(ruta)

		print("*Se guardo el mapa")
		return True
	
	def fuera_rango(self):
		if self.x_inicial>self.width or self.x_inicial<0 or self.x_final>self.width or self.x_final<0 or self.y_inicial>self.height or self.y_inicial<0 or self.y_final>self.height or self.y_final<0:
			return True
		else:
			return False
		
	def inicio_fin(self):
		print("-----------------------------------------------------")
		
		# Revisar si estan fuera de rango
		if not self.fuera_rango():

			
			print(f"Inicio:({self.x_inicial},{self.y_inicial})\nFin:({self.x_final},{self.y_final})")
			ruta=self.configuration_method()
			print("*Finalizado")
			ruta2 = []
			for i in range (len(ruta)):
				ruta2.append(ruta[i][0])
				ruta2.append(ruta[i][1])
			r = numpy.array (ruta2,dtype = numpy.float32)
			print (f"ruta=  {r}")


			scriptDir = os.path.dirname(__file__)
			ruta ="/home/sebastian/Uniandes202310/Robotica/proyecto_final/proyecto_final_ws/src/mi_robot_navegacion/mi_robot_navegacion/MapaRobotica.pgm"
		
			#ruta = scriptDir + "/Mapa_Test_drive_3_mod.jpg"
			image = cv2.imread(ruta)
				
			print("*Waiting for new coordenates")

		else:
			print()
			print("------------------------")
			print("Las coordenadas estan fuera de rango. Por favor ingrese nuevas coordenadas")
			print("------------------------")

	



def main(args=None):
    rclpy.init(args=args)
    mi_robot_ruta=planeacion()
    rclpy.spin(mi_robot_ruta)
    mi_robot_ruta.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
