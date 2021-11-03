#!/usr/bin/env python
import rospy
import sys
import time
from geometry_msgs.msg import *
from std_msgs.msg import *
#from termcolor import colored

#ROBOCOL
pos_x_info_anterior = 0
pos_y_info_anterior = 0
theta_info_anterior = 0
cont = 0
vel_izq = 0
vel_der = 0
rho_info, alpha_info = 0,0
pos_x_info, pos_y_info, theta_info = 0,0,0
pos_x_final_info, pos_y_final_info, theta_final_info, llegoAlaMeta = 0,0,0,0
puntos_faltantes_info = 0
probe_cont = 0
panic = False
vel_adjust = 0


def panic_callback(msg):
	global panic
	panic = msg.data

def cmd_vel_callback(msg):
	global vel_izq, vel_der
	vel_izq = msg[0]
	vel_der = msg[1]

def rho_callback(msg):
	global rho_info
	rho_info = msg.data

def alpha_callback(msg):
	global alpha_info
	alpha_info = msg.data

def pos_callback(msg):
	global pos_x_info_anterior, pos_y_info_anterior, theta_info_anterior, cont, pos_x_info, pos_y_info, theta_info

	pos_x_info = msg.linear.x
	pos_y_info = msg.linear.y
	theta_info = msg.angular.z

	#if (abs(pos_x_info-pos_x_info_anterior) < 0.01 and abs(pos_y_info-pos_y_info_anterior) < 0.01 and abs(theta_info-theta_info_anterior) < 0.01):
	#	cont = cont + 1
	#else:
	#	cont = 0

	pos_x_info_anterior = pos_x_info
	pos_y_info_anterior = pos_y_info
	theta_info_anterior = theta_info


def pos_final_callback(msg):
	global pos_x_final_info, pos_y_final_info, theta_final_info, puntos_faltantes_info, llegoAlaMeta
	pos_x_final_info = msg.linear.x
	pos_y_final_info = msg.linear.y
	theta_final_info = msg.angular.z
	puntos_faltantes_info = msg.angular.x
	llegoAlaMeta = msg.linear.z

def probe_callback(msg):
	global probe_cont
	probe_cont = msg.data

def vel_adjust_callback(msg):
	global vel_adjust
	vel_adjust = msg.data


def info_status():
	global vel_izq, vel_der, rho_info, pos_x_info_anterior, pos_y_info_anterior, theta_info_anterior, cont, pos_x_info, pos_y_info, theta_info, pos_x_final_info, pos_y_final_info, theta_final_info, llegoAlaMeta, panic, puntos_faltantes_info

	rospy.init_node('status', anonymous=True)  # Inicia el nodo status

	rospy.Subscriber("cmd_wheels_vel", Float32MultiArray, cmd_vel_callback, tcp_nodelay=True)
	rospy.Subscriber("Robocol/MotionControl/rho", Float32, rho_callback, tcp_nodelay=True)
	rospy.Subscriber("Robocol/MotionControl/alpha", Float32, alpha_callback, tcp_nodelay=True)
	rospy.Subscriber("Robocol/MotionControl/pos", Twist, pos_callback, tcp_nodelay=True)
	rospy.Subscriber("Robocol/MotionControl/pos_final", Twist, pos_final_callback, tcp_nodelay=True)
	rospy.Subscriber('probe_deployment_unit/probes_dropped', UInt8, probe_callback)
	rospy.Subscriber('Robocol/MotionControl/flag_panic', Bool, panic_callback, tcp_nodelay=True)
	rospy.Subscriber('Robocol/MotionControl/kp', Float32, vel_adjust_callback)
	
	rate = rospy.Rate(10)


	while not rospy.is_shutdown():
		mensaje = '______________________________ \n'
		mensaje = mensaje + 'Vel. Izq: ' + str(round(vel_izq,3)) + ' | Vel. Der: ' + str(round(vel_der,3)) + '\n'
		mensaje = mensaje + 'Rho: ' + str(round(rho_info,3)) + ' | Alpha: ' + str(round(alpha_info,3)) + '\n'
		mensaje = mensaje + 'pos. x: ' + str(pos_x_info) + ' | pos. y: ' + str(pos_y_info) + ' | theta: ' + str(theta_info) + '\n'
		mensaje = mensaje + 'pos. final x: ' + str(pos_x_final_info) + ' | pos. final y: ' + str(pos_y_final_info) + ' | theta final: ' + str(theta_final_info) + '\n'
		mensaje = mensaje + 'Droped probes: ' + str(probe_cont) + '\n'
		mensaje = mensaje + 'Puntos faltantes: ' + str(puntos_faltantes_info) + '\n'

		if (llegoAlaMeta == 1):
			mensaje = mensaje + 'YA LLEGO A LA META' + '\n'
		else:
			mensaje = mensaje + 'No ha llegado' + '\n'

		#if (cont >= 1000):
		if vel_izq < 0.01 and vel_izq > -0.01 and vel_der < 0.01:
			mensaje = mensaje + 'Detenido \n'
		else:
			mensaje = mensaje + 'En movimiento'
			if vel_izq < 0:
				mensaje = mensaje + ' - Voy hacia atras.'
			mensaje = mensaje + '\n'
		
		if vel_adjust != 0:
			mensaje = mensaje + 'La velocidad esta siendo ajustada por: ' + str(round(vel_adjust,3)) + '\n'

		if panic == True:
			mensaje = mensaje + 'Boton de PANICO \n'

		print(mensaje)
		
		#sys.stdout.write("\033[K") # Clear to the end of line
		#sys.stdout.write("\033[F") # Cursor up one line
		time.sleep(1)
		#sys.stdout.flush()
		#rate.sleep()
	#rate.sleep()


if __name__== '__main__':
	try:
		info_status()
	except rospy.ROSInterruptException:
		print('Nodo detenido')