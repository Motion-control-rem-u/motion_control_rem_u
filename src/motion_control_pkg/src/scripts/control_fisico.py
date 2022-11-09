#!/usr/bin/env python
import rospy
import numpy as np
import sys
import time
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from std_msgs.msg import *

global pos_x, pos_y, theta, deltaX, deltaY
global rate, rho, auto, ruta, hayRuta
global K_alpha, K_rho, K_beta, vel_adjust

#RECIBE LAS INDICACIONES PARA HACER EL RECORRIDO Y MUEVE AL ROBOT - VERSION 4
#ROBOCOL

pos_x, pos_y, theta = 0, 0, 0

deltaX, deltaY = 0, 0

rho = 0
auto = 0
hayRuta = 0 #poner en cero cuando se vaya a probar con planeacion
ruta = np.array([])
panic = False

K_rho = 0.10
K_alpha = 0.4
K_beta = -0.0
vel_adjust = 0.0

# Nuevo codigo para pasar a PWM
oldMax=50
oldMin=0
newMax=255
newMin=0

oldRange = (oldMax - oldMin)
newRange = (newMax - newMin) 

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
	global auto
	auto = msg.data
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

	endPos = [0,0,0] #Posicion final por defecto
	print('Starting control node...')
	print(' ')
	rospy.init_node('control', anonymous=True) #Inicio nodo

    # Publica en el topico pwm_data
	pub_traction_orders = rospy.Publisher("Robocol/MotionControl/pwm_data", Float32MultiArray, queue_size=10)
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	pub_probe = rospy.Publisher('probe_deployment_unit/drop', Empty, queue_size=1)
	pub_pos_status = rospy.Publisher('Robocol/MotionControl/pos', Twist, queue_size=10)
	pub_pos_final_status = rospy.Publisher('Robocol/MotionControl/pos_final', Twist, queue_size=10)
	pub_rho_status = rospy.Publisher('Robocol/MotionControl/rho', Float32, queue_size=10)
	pub_alpha_status = rospy.Publisher('Robocol/MotionControl/alpha', Float32, queue_size=10)

	rate = rospy.Rate(10) #10hz
	rospy.Subscriber("camera/odom/sample", Odometry, position_callback, tcp_nodelay=True)
	rospy.Subscriber('Robocol/MotionControl/flag_autonomo',Bool,habilitarMov, tcp_nodelay=True)
	rospy.Subscriber("Robocol/MotionControl/ruta", numpy_msg(Floats), ruta_callback, tcp_nodelay=True)
	rospy.Subscriber('Robocol/MotionControl/flag_panic', Bool, panic_callback, tcp_nodelay=True)
	rospy.Subscriber('Robocol/MotionControl/kp', Float32, vel_adjust_callback, tcp_nodelay=True)

	print("Paso pubs y susbs")
	#ruta = np.array([[-0.3,-0.3], [-0.3,0.3]])
	#ruta = np.array([[2.5,0.019],[1.5,1],[-0.035,0.0189]])

    # Referencia a envio de mensaje
	order = [0,0,0,0]
	pub_order = Float32MultiArray()
	
	l = 0.73

	vel_robot = Twist()
	pos_robot = Twist()
	pos_final_robot = Twist()

	print("paso inicializacion de variables")


	rho = np.sqrt(endPos[0]**2 + endPos[1]**2)
	alpha = -theta + np.arctan2(endPos[1], endPos[0])
	beta = -theta - alpha

	#auto = True

	while not rospy.is_shutdown():
		empezarDeNuevo = False
		v_vel = 0
		w_vel = 0
		vel_robot.linear.x = 0
		vel_robot.linear.y = 0
		vel_robot.angular.z = 0
		pub.publish(vel_robot)
		

		left_u=0
		right_u=0
		left_d=0
		right_d=0
		order[0],order[1],order[2], order[3] = left_u,right_u,left_d,right_d
        

		pub_order.data = order
		pub_traction_orders.publish(pub_order)
		
		# v_x = 0
		# v_y = 0
		# v_omega = 0
		# print("ruta")
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
			print("segundo while")
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
				#revisar si es pi o 2pi
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
				#Siempre que cambio de autonomo a manual, le digo al rover que vuelva a calcular rho y alpha, ya qu eme movi
				while empezarDeNuevo == True:
					while abs(alpha) > 0.05:
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

							v_omega = w_vel
							vel_robot.linear.x = 0
							vel_robot.linear.y = 0
							vel_robot.angular.z = v_omega +vel_adjust
							pub.publish(vel_robot)

                            # Publicacion a velocidades pwm de cada llanta

                            
							left_u=-(w_vel*l)/2
							right_u=(w_vel*l)/2
							left_d=-(w_vel*l)/2
							right_d=(w_vel*l)/2

							# Hacer cambio a la escala de PWM (255)
							left_u = (((left_u - oldMin) * newRange) / oldRange) + newMin
							right_u = (((right_u - oldMin) * newRange) / oldRange) + newMin
							left_d = (((left_d - oldMin) * newRange) / oldRange) + newMin
							right_d = (((right_d - oldMin) * newRange) / oldRange) + newMin

							# Revisar que el valor sea mayor o igual a -255
							left_u=max(left_u,-255)
							right_u=max(right_u,-255)
							left_d=max(left_d,-255)
							right_d=max(right_d,-255)

							# Revisar que el valor sea mayor o igual a -255
							left_u=min(left_u,255)
							right_u=min(right_u,255)
							left_d=min(left_d,255)
							right_d=min(right_d,255)

							order[0],order[1],order[2], order[3] = left_u,right_u,left_d,right_d

							pub_order.data = order
							pub_traction_orders.publish(pub_order)

                            # Sigue codigo normal

							pub_alpha_status.publish(alpha)

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
					while rho > 0.05 and empezarDeNuevo == False:
						if auto == True:
							
							print('EndPos: ' + str(endPos[0]) + ' ' + str(endPos[1]) + ' ' + str(round(endPos[2],3)) + ' | RHO: ' + str(round(rho,3)) + ' | alfa: ' + str(round(alpha,3)) + ' | Pose: ' + str(round(pos_x,3)) + ', ' + str(round(pos_y,3)) + ', ' + str(round(theta,3)))#+ ' | v_omega: ' + str(round(v_omega,3)))
							sys.stdout.write("\033[K") # Clear to the end of line
							sys.stdout.write("\033[F") # Cursor up one line
							time.sleep(1)
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

							#v_vel = K_rho * np.exp(rho) - K_rho #lento
							#v_vel = K_rho * np.log(rho/50) + 0.8 


							w_vel = K_alpha*alpha + K_beta*beta
							
							if alpha <= np.pi/2 and alpha > -np.pi/2:
								v_x = v_vel
								v_y = 0
								v_omega = w_vel
							else:# (np.abs(alpha) >= np.pi and np.abs(alpha) < 3*np.pi/2) or (alpha <= np.pi and alpha > np.pi/2):
								v_x = -v_vel
								v_y = 0
								v_omega = 0
								#print('----------------Voy Hacia Atras--------------------V_X: ' + str(round(v_x, 3)))
							
							vel_robot.linear.x = v_x + vel_adjust
							vel_robot.linear.y = v_y
							vel_robot.angular.z = v_omega
							#vel_robot.angular.z = 0
							pub.publish(vel_robot)


                            # Publicacion a velocidades pwm de cada llanta

                            
							left_u=v_x + vel_adjust -(w_vel*l)/2
							right_u=v_x + vel_adjust+(w_vel*l)/2
							left_d=v_x + vel_adjust-(w_vel*l)/2
							right_d=v_x + vel_adjust+(w_vel*l)/2

							# Hacer cambio a la escala de PWM (255)
							left_u = (((left_u - oldMin) * newRange) / oldRange) + newMin
							right_u = (((right_u - oldMin) * newRange) / oldRange) + newMin
							left_d = (((left_d - oldMin) * newRange) / oldRange) + newMin
							right_d = (((right_d - oldMin) * newRange) / oldRange) + newMin

							# Revisar que el valor sea mayor o igual a -255
							left_u=max(left_u,-255)
							right_u=max(right_u,-255)
							left_d=max(left_d,-255)
							right_d=max(right_d,-255)

							# Revisar que el valor sea mayor o igual a -255
							left_u=min(left_u,255)
							right_u=min(right_u,255)
							left_d=min(left_d,255)
							right_d=min(right_d,255)

							order[0],order[1],order[2], order[3] = left_u,right_u,left_d,right_d

							pub_order.data = order
							pub_traction_orders.publish(pub_order)

                            # Sigue codigo normal



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
				vel_robot.linear.x = v_x 
				vel_robot.linear.y = v_y
				vel_robot.angular.z = v_omega
				pub.publish(vel_robot)
				pub_alpha_status.publish(alpha)

                # Publicacion a velocidades pwm de cada llanta

                
				left_u=v_x  -(w_vel*l)/2
				right_u=v_x +(w_vel*l)/2
				left_d=v_x -(w_vel*l)/2
				right_d=v_x +(w_vel*l)/2

				# Hacer cambio a la escala de PWM (255)
				left_u = (((left_u - oldMin) * newRange) / oldRange) + newMin
				right_u = (((right_u - oldMin) * newRange) / oldRange) + newMin
				left_d = (((left_d - oldMin) * newRange) / oldRange) + newMin
				right_d = (((right_d - oldMin) * newRange) / oldRange) + newMin

				# Revisar que el valor sea mayor o igual a -255
				left_u=max(left_u,-255)
				right_u=max(right_u,-255)
				left_d=max(left_d,-255)
				right_d=max(right_d,-255)

				# Revisar que el valor sea mayor o igual a -255
				left_u=min(left_u,255)
				right_u=min(right_u,255)
				left_d=min(left_d,255)
				right_d=min(right_d,255)

				order[0],order[1],order[2], order[3] = left_u,right_u,left_d,right_d

				pub_order.data = order
				pub_traction_orders.publish(pub_order)

                # Sigue codigo normal

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
        #rate.sleep()
if __name__ == '__main__':
	try:
		main_control()
	except rospy.ROSInterruptException:
		print('Nodo detenido')


