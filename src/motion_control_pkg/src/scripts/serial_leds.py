#!/usr/bin/python

import rospy
from std_msgs.msg import Float32MultiArray, Bool, Int16MultiArray
import serial
import serial.tools.list_ports
import time

# OBTIENE PWM DE CONTROL AUTONOMO O DE MANUAL Y LO ESCRIBE POR SERIAL.
# Corre en la Jetson
# funciona con joystick _publisher.py y control_rem.py
# ROBOCOL 2021-2

flag_autonomo = Bool()
(
    vel_izq_u,
    vel_der_u,
    vel_izq_d,
    vel_der_d,
    auto_vel_izq,
    auto_vel_der,
    ) = (0, 0, 0, 0, 0, 0)
#Modo para funcionamiento de drivers (d=drive, b=break, r=reverse)
modo = 'd'
auto = False    
puerto = ''
led = 'V'

# Verificamos el puerto del arduino para hacer la conexion

# ports = list(serial.tools.list_ports.comports())
# print(len(ports) )
# for p in ports:
#     print(p.description)
#     if 'ttyACM0' in p.description:
#         print ('This is an Arduino!')
#         puerto = p.description

# Crea la conexion serial con el Arduino

arduino = serial.Serial('/dev/ttyACM0' + str(puerto), 115200, timeout=10)

# arduino = serial.Serial('ttyACMO', 115200, timeout=10)

def habilitarMov(msg):  # Me indica si debo mover el robot autonomamente o no
    global auto, arduino, uso_arduino
    auto = msg.data

    print ('auto:' + str(auto))


def auto_vel_callback(msg):
    global auto_vel_izq, auto_vel_der
    auto_vel_izq = msg.data[0]
    auto_vel_der = msg.data[1]


def manual_vel_callback(msg):
    # izquierdo_arriba, derecho_arriba, izquierdo_abajo, derecho_abajo
    global vel_izq_u, vel_der_u, vel_izq_d, vel_der_d
    vel_izq_u = int(msg.data[0])
    vel_der_u = int(msg.data[1])
    vel_izq_d = int(msg.data[2])
    vel_der_d = int(msg.data[3])

# Agregar ceros
def agregar_ceros(numero):
    es_positivo=numero>=0
    numero_str=str(abs(numero))
    if es_positivo:
        numero_str="0"*(4-len(numero_str))+numero_str
    else:
        numero_str="-"+"0"*(3-len(numero_str))+numero_str
    return numero_str


def rover_serial_writer():
    print ('Starting Arduino Serial Comunication node... \n')
    rospy.init_node('rover_teleop', anonymous=True)  # Inicia el nodo teleop

    rospy.Subscriber('Robocol/MotionControl/cmd_wheels_vel',
                    # Control
                     Float32MultiArray, manual_vel_callback,
                     tcp_nodelay=True)
    rospy.Subscriber('Robocol/MotionControl/pwm_data',
                    # joystick
                     Float32MultiArray, manual_vel_callback,
                     tcp_nodelay=True)
    rospy.Subscriber('Robocol/MotionControl/flag_autonomo', Bool,
                     habilitarMov, tcp_nodelay=True)

    rate = rospy.Rate(5)

    order = [0, 0, 0, 0, modo]
    (left_u, right_u, left_d, right_d) = (0, 0, 0, 0)
    print ('Waiting for flag_autonomo...')
    while not rospy.is_shutdown():

        # # Si estamos en modo manual, usamos lo que manda el joystick
        # if auto == False:
        #     left_u = vel_izq_u
        #     right_u = vel_der_u
        #     left_d = vel_izq_d
        #     right_d = vel_der_d
        # else:

        # # Si estamos en modo automatico, usamos lo que manda el control

        #     left_u = auto_vel_izq
        #     right_u = auto_vel_der
        #     left_d = auto_vel_izq
        #     right_d = auto_vel_der

        left_u = vel_izq_u
        right_u = vel_der_u
        left_d = vel_izq_d
        right_d = vel_der_d

        #print ((left_u, right_u,left_d,right_d))

        left_u =agregar_ceros(left_u)
        right_u =agregar_ceros(right_u)
        left_d =agregar_ceros(left_d)
        right_d =agregar_ceros(right_d)

        # left = str(left)
        # right = str(right)
        # len_left = len(left)
        # len_right = len(right)

        # if len_left == 1:
        #     left = '000' + left
        # elif len_left == 2:
        #     if left[0] == '-':
        #         left = '-00' + left[1]
        #     else:
        #         left = '00' + left
        # elif len_left == 3:
        #     if left[0] == '-':
        #         left = '-0' + left[1] + left[2]
        #     else:
        #         left = '0' + left

        # if len_right == 1:
        #     right = '000' + right
        # elif len_right == 2:
        #     if right[0] == '-':
        #         right = '-00' + right[1]
        #     else:
        #         right = '00' + right
        # elif len_right == 3:
        #     if right[0] == '-':
        #         right = '-0' + right[1] + right[2]
        #     else:
        #         right = '0' + right
        
        if auto == True:
            led = 'A'
        if auto == False:
            led = 'R'
        #time.sleep(0.8)
        (order[0], order[1],order[2],order[3]) = (left_u, right_u, left_d, right_d)
        encoded = (str(order) +led+ '\n').encode('utf-8')
        print(left_u,right_u,left_d,right_d)

        arduino.write(encoded)

        #print(encoded)

        rate.sleep()


    # rate.sleep()

if __name__ == '__main__':
    try:
        rover_serial_writer()
    except rospy.ROSInterruptException:
        print ('Nodo detenido')
