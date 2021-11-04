#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import Float32MultiArray, Bool, Int32MultiArray
import serial
import time

#OBTIENE PWM DE CONTROL AUTONOMO O DE MANUAL Y LO ESCRIBE POR SERIAL.
#Corre en la Jetson
#funciona con joystick _publisher.py y control_rem.py
#ROBOCOL 2021-2

flag_autonomo = Bool()
auto_vel_izq, auto_vel_der = 0,0
modo = "d"

#Crea la conexion serial con el Arduino
arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=10)

def habilitarMov(msg): #Me indica si debo mover el robot autonomamente o no
	global auto,arduino,uso_arduino
	auto = msg.data

	print('auto:' + str(auto))

def auto_vel_callback(msg):
	global auto_vel_izq, auto_vel_der
	auto_vel_izq = msg[0]
	auto_vel_der = msg[1]

def manual_vel_callback(msg):
	global vel_izq, vel_der
	vel_izq = msg[0]
	vel_der = msg[1]


def rover_serial_writer():
	print('Starting rover_teleop node and publishing flag_autonomo...')
	rospy.init_node('rover_teleop', anonymous=True)  # Inicia el nodo teleop

	rospy.Subscriber("Robocol/MotionControl/cmd_wheels_vel", Float32MultiArray, auto_vel_callback, tcp_nodelay=True)
	rospy.Subscriber("Robocol/MotionControl/pwm_data", Int32MultiArray, manual_vel_callback, tcp_nodelay=True)
	rospy.Subscriber('Robocol/MotionControl/flag_autonomo',Bool,habilitarMov, tcp_nodelay=True)

	rate = rospy.Rate(10)

	order = [0,0,modo]
	left,right = 0,0

	while not rospy.is_shutdown():

		#Si estamos en modo manual, usamos lo que manda el joystick
		if auto == False:
			left = vel_izq
			right = vel_der
		#Si estamos en modo automatico, usamos lo que manda el control
		else:
			left = auto_vel_izq
			right = auto_vel_izq
		
		left = str(left)
		right = str(right)
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
		arduino.write(encoded)
		rate.sleep()
	#rate.sleep()


if __name__== '__main__':
	try:
		rover_serial_writer()
	except rospy.ROSInterruptException:
		print('Nodo detenido')