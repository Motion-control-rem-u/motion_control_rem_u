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
    vel_izq,
    vel_der,
    vel_izq_a,
    vel_der_a,
    auto_vel_izq,
    auto_vel_der,
    ) = (0, 0, 0, 0)
modo = 'd'
auto = False
puerto = ''

# Verificamos el puerto del arduino para hacer la conexion

ports = list(serial.tools.list_ports.comports())
for p in ports:
    print(p.description)
    if 'ttyACM' in p.description:
        print ('This is an Arduino!')
        puerto = p.description

# Crea la conexion serial con el Arduino

arduino = serial.Serial('/dev/' + str(puerto), 115200, timeout=10)


def habilitarMov(msg):  # Me indica si debo mover el robot autonomamente o no
    global auto, arduino, uso_arduino
    auto = msg.data

    print ('auto:' + str(auto))


def auto_vel_callback(msg):
    global auto_vel_izq, auto_vel_der
    auto_vel_izq = msg.data[0]
    auto_vel_der = msg.data[1]


def manual_vel_callback(msg):
    global vel_izq, vel_der, vel_izq_a, vel_der_a
    vel_izq = msg.data[0]
    vel_der = msg.data[1]
    vel_izq_a = msg.data[2]
    vel_der_a = msg.data[3]

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
                     Float32MultiArray, auto_vel_callback,
                     tcp_nodelay=True)
    rospy.Subscriber('Robocol/MotionControl/pwm_data',
                     Float32MultiArray, manual_vel_callback,
                     tcp_nodelay=True)
    rospy.Subscriber('Robocol/MotionControl/flag_autonomo', Bool,
                     habilitarMov, tcp_nodelay=True)

    rate = rospy.Rate(10)

    order = [0, 0, modo]
    (left, right, left_a, right_a) = (0, 0, 0, 0)
    print ('Waiting for flag_autonomo...')
    while not rospy.is_shutdown():

        # Si estamos en modo manual, usamos lo que manda el joystick

        if auto == False:
            left = vel_izq
            right = vel_der
            left_a = vel_izq_a
            right_a = vel_der_a
        else:

        # Si estamos en modo automatico, usamos lo que manda el control

            left = auto_vel_izq
            right = auto_vel_izq
        print ((left, right))

        left = str(left)
        right = str(right)
        len_left = len(left)
        len_right = len(right)

        if len_left == 1:
            left = '000' + left
        elif len_left == 2:
            if left[0] == '-':
                left = '-00' + left[1]
            else:
                left = '00' + left
        elif len_left == 3:
            if left[0] == '-':
                left = '-0' + left[1] + left[2]
            else:
                left = '0' + left

        if len_right == 1:
            right = '000' + right
        elif len_right == 2:
            if right[0] == '-':
                right = '-00' + right[1]
            else:
                right = '00' + right
        elif len_right == 3:
            if right[0] == '-':
                right = '-0' + right[1] + right[2]
            else:
                right = '0' + right
        
        
        left_a =agregar_ceros(left_a)
        right_a =agregar_ceros(right_a)
        

        time.sleep(0.8)
        (order[0], order[1],order[2],order[3]) = (left, right, left_a, right_a)
        encoded = (str(order) + '\n').encode('utf-8')
        print(left_a,right_a)

        #arduino.write(encoded)

        # print(encoded)

        rate.sleep()


    # rate.sleep()

if __name__ == '__main__':
    try:
        rover_serial_writer()
    except rospy.ROSInterruptException:
        print ('Nodo detenido')
