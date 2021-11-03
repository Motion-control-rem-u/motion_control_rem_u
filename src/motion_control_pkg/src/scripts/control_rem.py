#!/usr/bin/env python
import rospy
import numpy as np
import sys
import time
import serial
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from std_msgs.msg import *

global pos_x, pos_y, theta, deltaX, deltaY
global rate, rho, auto, ruta, hayRuta
global K_alpha, K_rho, K_beta, vel_adjust

#Implementacion para moverlo con Arduinos - Tatacoa 2021-2
#RECIBE LAS INDICACIONES PARA HACER EL RECORRIDO Y MUEVE AL ROBOT - VERSION 4
#ROBOCOL

modo="d" #Modo para funcionamiento de drivers (d=drive, b=break, r=reverse)

pos_x, pos_y, theta = 0, 0, 0

deltaX, deltaY = 0, 0

rho = 0
auto = 0
uso_arduino = 0
arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=10)
arduino.close()
hayRuta = 1 #poner en cero cuando se vaya a probar con planeacion
ruta = np.array([])
panic = False

K_rho = 0.10
K_alpha = 0.4
K_beta = -0.0
vel_adjust = 0.0

def position_callback(msg): #Me regresa la posicion en el marco inercial del robot
	global rate, pos_x, pos_y, theta

	pos_x = msg.pose.pose.position.x
	pos_y = msg.pose.pose.position.y
	pos_z = msg.pose.pose.position.z

	orC_x = msg.pose.pose.orientation.x
	orC_y = msg.pose.pose.orientation.y
	orC_z = msg.pose.pose.orientation.z
	orC_w = msg.pose.pose.orientation.w


	#orientation_q = msg.pose.pose.orientation
	orientation_list = [orC_x, orC_y, orC_z, orC_w]
	(roll, pitch,theta) = euler_from_quaternion(orientation_list)

def panic_callback(msg):
	panic = msg.data

def vel_adjust_callback(msg):
	global vel_adjust
	vel_adjust = msg.data	
	print('Se ajusta la velocidad por: ',round(vel_adjust,3))

def habilitarMov(msg): #Me indica si debo mover el robot autonomamente o no
	global auto,arduino,uso_arduino
	auto = msg.data
	if auto == 1 and uso_arduino == 0:
		uso_arduino = 1
		arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=10)
	else:
		uso_arduino = 0
		arduino.close()

	print('auto:' + str(auto))


def ruta_callback(msg):
	global ruta, hayRuta
	hayRuta = 1
	#a = np.zeros((3,3))
	print('msg.data y len:')
	print(msg.data)
	
	aux = msg.data
	ruta = aux.copy()

	ruta.resize(len(msg.data)/2, 2)
	print('ruta: ')
	print(ruta)
	print(len(ruta))

#Funcion principal de movimiento
def main_control():
	global pos_x, pos_y, theta, deltaX, deltaY, rho, rate, auto, hayRuta, ruta, K_alpha, K_rho, K_beta

	l = 0.23 #metros, separacion ruedas
	r = 0.07/2 #metros, radio ruedas

	endPos = [0,0,0] #Posicion final por defecto
	print('Starting control node...')
	print(' ')
	rospy.init_node('control', anonymous=True) #Inicio nodo

	#Publica las velocidades para ruedas a la derecha e izquierda.
	pub = rospy.Publisher('cmd_wheels_vel', Float32MultiArray, queue_size=10)
	pub_probe = rospy.Publisher('probe_deployment_unit/drop', Empty, queue_size=1)
	pub_pos_status = rospy.Publisher('Robocol/MotionControl/pos', Twist, queue_size=10)
	pub_pos_final_status = rospy.Publisher('Robocol/MotionControl/pos_final', Twist, queue_size=10)
	pub_rho_status = rospy.Publisher('Robocol/MotionControl/rho', Float32, queue_size=10)
	pub_alpha_status = rospy.Publisher('Robocol/MotionControl/alpha', Float32, queue_size=10)

	rate = rospy.Rate(10) #10hz
	rospy.Subscriber("/camera/odom/sample", Odometry, position_callback, tcp_nodelay=True)
	rospy.Subscriber('Robocol/MotionControl/flag_autonomo',Bool,habilitarMov, tcp_nodelay=True)
	rospy.Subscriber("Robocol/MotionControl/ruta", numpy_msg(Floats), ruta_callback, tcp_nodelay=True)
	rospy.Subscriber('Robocol/MotionControl/flag_panic', Bool, panic_callback, tcp_nodelay=True)
	rospy.Subscriber('Robocol/MotionControl/kp', Float32, vel_adjust_callback, tcp_nodelay=True)

	#ruta = np.array([[-0.3,-0.3], [-0.3,0.3]])
	ruta = np.array([[2,0.0],[2,1],[0,1]])

	vel_robot = Float32MultiArray()
	pos_robot = Twist()
	pos_final_robot = Twist()

	#Esta variable se encargara de saber si debemos abrir o no el puerto serial de arduino
	uso_arduino = 0

	rho = np.sqrt(endPos[0]**2 + endPos[1]**2)
	alpha = -theta + np.arctan2(endPos[1], endPos[0])
	beta = -theta - alpha

	# Referencia a envio de mensaje
	order = [0,0,modo]
	left,right = 0,0

	#auto = True #Desactivar para cuando se quiera correr con teclado.py

	while not rospy.is_shutdown():

		#Esta variable se encarga de que cuando se cambie entre joystick y autonomo, el robot siempre vuelva a 
		#ajustar el angulo sin importar si antes del cambio estaba ajustando rho
		empezarDeNuevo = False
		v_vel = 0
		w_vel = 0
		vr, vl = 0,0
		vel_robot.data = [vr,vl]
		#pub.publish(vel_robot)

		print('Hay ruta?  ' + ('Si' if hayRuta == 1 else 'No'))
		
		print('Ruta: ' + str(ruta))

		pos_robot.linear.x = round(pos_x,3)
		pos_robot.linear.y = round(pos_y,3)
		pos_robot.angular.z = round(theta,3)
		pub_pos_status.publish(pos_robot)


		pos_final_robot.linear.x = round(endPos[0],3)
		pos_final_robot.linear.y = round(endPos[1],3)
		pos_final_robot.angular.z = round(endPos[2],3)
		pub_pos_final_status.publish(pos_final_robot)

		
		
		while hayRuta == 1:
			pos_final_robot.linear.z = 0 #Indica que no ha llegado al destino.
			cont_puntos_destino = 0
			print('Esperando comando de flag_autonomo...')
			for coord in ruta:
				coord_x = coord[0]
				coord_y = coord[1]
				auxtheta = np.arctan2(float(coord_y), float(coord_x))
				endPos = [float(coord_x), float(coord_y), float(auxtheta) ] # [X. Y, THETA]

				deltaX = endPos[0] - pos_x
				deltaY = endPos[1] - pos_y
				deltatheta = endPos[2] - theta

				rho = np.sqrt(deltaX**2 + deltaY**2)
				alpha = -theta + np.arctan2(deltaY, deltaX)
				if alpha > np.pi:
					alpha = alpha - np.pi 
				elif alpha < -np.pi:
					alpha= alpha +np.pi

				beta = 0

				pos_robot.linear.x = round(pos_x,3)
				pos_robot.linear.y = round(pos_y,3)
				pos_robot.angular.z = round(theta,3)
				pub_pos_status.publish(pos_robot)

				pos_final_robot.linear.x = round(endPos[0],3)
				pos_final_robot.linear.y = round(endPos[1],3)
				pos_final_robot.angular.z = round(endPos[2],3)
				pos_final_robot.angular.x = len(ruta) - cont_puntos_destino #Para no usar mas publishers, se usara esto para indicar el num. de puntos que faltan
				cont_puntos_destino = cont_puntos_destino + 1
				pub_pos_final_status.publish(pos_final_robot)
				

				empezarDeNuevo = True
				while empezarDeNuevo == True:
					while abs(alpha) > 0.15:
						if auto == True:

							print('EndPos: ' + str(endPos[0]) + ' ' + str(endPos[1]) + ' ' + str(round(endPos[2],3)) + ' | rho: ' + str(round(rho,3)) + ' | ALFA: ' + str(round(alpha,3)) + ' | Pose: ' + str(round(pos_x,3)) + ', ' + str(round(pos_y,3)) + ', ' + str(round(theta,3)))
							sys.stdout.write("\033[K") # Clear to the end of line
							sys.stdout.write("\033[F") # Cursor up one line
							time.sleep(1)
							deltaX = endPos[0] - pos_x
							deltaY = endPos[1] - pos_y
							deltatheta = endPos[2] - theta
							
							aux = theta
							if aux < 0:
								aux = np.pi*2 + aux

							#alpha = -aux + beta

							alpha = -theta + np.arctan2(deltaY, deltaX)
							if alpha > np.pi:
								alpha = alpha - np.pi 
							elif alpha < -np.pi:
								alpha= alpha +np.pi
							K_alpha = 0.4 + 0.3 * np.exp(-alpha)

							w_vel = K_alpha*alpha + K_beta*beta

							vel_robot.data= [w_vel, -w_vel] #CAMBIAR ESTO ESTA MAL]
							pub.publish(vel_robot)
							pub_alpha_status.publish(alpha)

							#Procedimiento para codificar el mensaje y enviarlo a Arduino.
							left = str(w_vel + vel_adjust)
							right = str(-w_vel + vel_adjust)
							len_left = len(left)
							len_right = len(right)

							if(len_left==1):
								left = "000"+left
							elif(len_left == 2):
								if (left[0]=="-"):
									left = "-00" + left[1]
								else:
									left = "00" + left
							elif(len_left == 3):
								if (left[0]=="-"):
									left = "-0" + left[1]+left[2]
								else:
									left = "0" + left

							if(len_right==1):
								right = "000"+right
							elif(len_right == 2):
								if (right[0]=="-"):
									right = "-00" + right[1]
								else:
									right = "00" + right
							elif(len_right == 3):
								if (right[0]=="-"):
									right = "-0" + right[1]+right[2]
								else:
									right = "0" + right

							time.sleep(0.5)
							order[0], order[1] = left, right
							encoded = (str(order)+"\n").encode('utf-8')
							#print(encoded)
							if uso_arduino == 1:
								arduino.write(encoded)

							pos_robot.linear.x = round(pos_x,3)
							pos_robot.linear.y = round(pos_y,3)
							pos_robot.angular.z = round(theta,3)
							pub_pos_status.publish(pos_robot)

							pos_final_robot.linear.x = round(endPos[0],3)
							pos_final_robot.linear.y = round(endPos[1],3)
							pos_final_robot.angular.z = round(endPos[2],3)
							pub_pos_final_status.publish(pos_final_robot)

						else:
							print('Estamos en modo manual.')
							sys.stdout.write("\033[K") # Clear to the end of line
							sys.stdout.write("\033[F") # Cursor up one line
							time.sleep(1)
							rate.sleep()
					K_alpha = 0.35
					empezarDeNuevo = False
					while rho > 0.1 and empezarDeNuevo == False:
						if auto == True:

							deltaX = endPos[0] - pos_x
							deltaY = endPos[1] - pos_y
							deltatheta = endPos[2] - theta

							rho = np.sqrt(deltaX**2 + deltaY**2)
							alpha = -theta + np.arctan2(deltaY, deltaX)
							if alpha > np.pi:
								alpha = alpha - np.pi 
							elif alpha < -np.pi:
								alpha= alpha +np.pi

							v_vel = K_rho*rho + 0.5 * np.exp(-rho)

							if rho < 0.8:
								v_vel = (K_rho*(1-0.5))*rho + 0.4 * np.exp(-rho)

							if rho < 0.4:
								v_vel = (K_rho*(1-0.85))*rho + 0.1 * np.exp(-rho)

							w_vel = K_alpha*alpha + K_beta*beta
							
							if alpha <= np.pi/2 and alpha > -np.pi/2:
								vr = (v_vel + w_vel*(l/2))/r
								vl = (v_vel - w_vel*(l/2))/r
							else:# (np.abs(alpha) >= np.pi and np.abs(alpha) < 3*np.pi/2) or (alpha <= np.pi and alpha > np.pi/2):
								vr = -((v_vel + w_vel*(l/2))/r)
								vl = -((v_vel - w_vel*(l/2))/r)
								#print('----------------Voy Hacia Atras--------------------V_X: ' + str(round(v_x, 3)))
							
							vel_robot.data = [vl + vel_adjust, vr + vel_adjust]

							#Procedimiento para codificar el mensaje y enviarlo a Arduino.
							left = str(vl + vel_adjust)
							right = str(vr + vel_adjust)
							len_left = len(left)
							len_right = len(right)

							if(len_left==1):
								left = "000"+left
							elif(len_left == 2):
								if (left[0]=="-"):
									left = "-00" + left[1]
								else:
									left = "00" + left
							elif(len_left == 3):
								if (left[0]=="-"):
									left = "-0" + left[1]+left[2]
								else:
									left = "0" + left

							if(len_right==1):
								right = "000"+right
							elif(len_right == 2):
								if (right[0]=="-"):
									right = "-00" + right[1]
								else:
									right = "00" + right
							elif(len_right == 3):
								if (right[0]=="-"):
									right = "-0" + right[1]+right[2]
								else:
									right = "0" + right

							time.sleep(0.5)
							order[0], order[1] = left, right
							encoded = (str(order)+"\n").encode('utf-8')
							#print(encoded)
							if uso_arduino == 1:
								arduino.write(encoded)

							pub.publish(vel_robot)
							pub_alpha_status.publish(alpha)
							pub_rho_status.publish(rho) #Se publica el rho del robot a status

							#Se publica la posicion del robot a status
							pos_robot.linear.x = round(pos_x,3)
							pos_robot.linear.y = round(pos_y,3)
							pos_robot.angular.z = round(theta,3)
							pub_pos_status.publish(pos_robot)

							pos_final_robot.linear.x = round(endPos[0],3)
							pos_final_robot.linear.y = round(endPos[1],3)
							pos_final_robot.angular.z = round(endPos[2],3)
							pub_pos_final_status.publish(pos_final_robot)

							print('EndPos: ' + str(endPos[0]) + ' ' + str(endPos[1]) + ' ' + str(round(endPos[2],3)) + ' | RHO: ' + str(round(rho,3)) + ' | alfa: ' + str(round(alpha,3)) + ' | Pose: ' + str(round(pos_x,3)) + ', ' + str(round(pos_y,3)) + ', ' + str(round(theta,3))+ ' | vel_robot: ' + str(vel_robot.data))
							sys.stdout.write("\033[K") # Clear to the end of line
							sys.stdout.write("\033[F") # Cursor up one line
							time.sleep(1)


							rate.sleep()
						else:
							while auto == 1:
								#print('auto: ' + str(auto))
								print('Estamos en modo manual.')
								sys.stdout.write("\033[K") # Clear to the end of line
								sys.stdout.write("\033[F") # Cursor up one line
								time.sleep(1)
								rate.sleep()
							empezarDeNuevo = True
			print("Probe droped")
			pub_probe.publish()
			hayRuta = 0

			if auto == True:
				print("Ya llegue al destino")
				vel_robot.data = [0,0]
				pub.publish(vel_robot)
				pub_alpha_status.publish(alpha)

				#Se publica la posicion del robot a status
				pos_robot.linear.x = round(pos_x,3)
				pos_robot.linear.y = round(pos_y,3)
				pos_robot.angular.z = round(theta,3)
				pub_pos_status.publish(pos_robot)

				pos_final_robot.linear.x = round(endPos[0],3)
				pos_final_robot.linear.y = round(endPos[1],3)
				pos_final_robot.angular.z = round(endPos[2],3)
				pos_final_robot.linear.z = 1 #Indica que ya ha llegado al destino.
				pos_final_robot.angular.x = len(ruta) - cont_puntos_destino #Para no usar mas publishers, se usara esto para indicar el num. de puntos que faltan
				
				pub_pos_final_status.publish(pos_final_robot)

			else:
				print('Estamos en modo manual.')

			rate.sleep()
		rate.sleep()
if __name__ == '__main__':
	try:
		main_control()
	except rospy.ROSInterruptException:
		print('Nodo detenido')
