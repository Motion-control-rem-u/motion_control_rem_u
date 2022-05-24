#!/usr/bin/env python3
import rospy
import sys
from std_msgs.msg import Bool

#MANEJA EL ROBOT CON EL TECLADO U OPRIMIENDO K CAMBIA AL CONTROL AUTOMATICO
#DESPUES DE CORRER ESTE CORRER EL NODO DE control_sim
#ROBOCOL

flag_autonomo = Bool()
panic = False


def panic_callback(msg):
	global panic
	panic = msg.data


def rover_teleop():
	global panic
	print('Starting rover_teleop node and publishing flag_autonomo...')
	rospy.init_node('rover_teleop', anonymous=True)  # Inicia el nodo teleop
	pub_flagAuto = rospy.Publisher('Robocol/MotionControl/flag_autonomo', Bool, queue_size=1)
	rospy.Subscriber('Robocol/MotionControl/flag_panic', Bool, panic_callback, tcp_nodelay=True)

	rate = rospy.Rate(10)

	
	modo = 0
	print('Se inicializa con movimiento teledirigo/manual')
	pub_flagAuto.publish(False)
	print(' ')
	print('Para cambiar entre autonomo y manual oprima k')
	print('Esperando comando...')
	print(' ')
	panico_des = False
	while not rospy.is_shutdown():
		if panic == True:
			if panico_des == False:
				print('Boton de PANICO Activo - Desactivar Boton de panico para movimiento.')
				panico_des = True
		else:
			if panico_des == True:
				panico_des = False
				print('Boton de PANICO desactivado.')
				print('Esperando comando...')

			movimiento = input()
			sys.stdout.write("\033[K") # Clear to the end of line
			sys.stdout.write("\033[F") # Cursor up one line
			if movimiento == "k":
				
				if modo == 0:
					modo = 1
					flag_autonomo = False
				else:
					modo = 0
					flag_autonomo = True

				pub_flagAuto.publish(flag_autonomo)
				print('\r modo: ' + ('Teleop..' if modo == 1 else 'Autonomo'))
				sys.stdout.write("\033[K") # Clear to the end of line
				sys.stdout.write("\033[F") # Cursor up one line

		rate.sleep()
	#rate.sleep()


if __name__== '__main__':
	try:
		rover_teleop()
	except rospy.ROSInterruptException:
		print('Nodo detenido')