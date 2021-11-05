#!/usr/bin/env python3

import rospy
import math
import time
from std_msgs.msg import Int16MultiArray, Float32MultiArray
import pygame

#PERMITE USAR EL JOYSTICK Y PUBLICA LOS PWM AL ARDUINO.
#Corre en el PC
#Funciona con control_rem.py y serial_writer.py
#ROBOCOL - 2021-2

auto = 1
uso_arduino = 0

modo="d"

# Referencia del objeto del joystick
joystick_ref = None
# Referencia de eje #0 del joystick
axis0 = 0
# Referencia de eje #1 del joystick
axis1 = 0
# Referencia de eje #2 del joystick
axis2 = 0
# Referencia de eje #3 del joystick
axis3 = 0
# Referenc de eventos generados de eje Movido:
axis_moved = False
# Maximas rpm de las llantas
max_rpm = 250
# Deadzone de la rotacion del joystick
deadzone_rot = 0.30
#Deadzone del moviemiento del joystick
deadzone_stick = 0.10

# Caracteristicas del Joystick
# Numero de botones = 13
# Numero de ejes 4

# Eje 0 : Horizontal Joystick: Derecha 1, Izquierda -1
# Eje 1 : Vertical Joystick: Atras 1, Frente -1
# Eje 2 : Rotacional Joystick: En contra manecillas -1, A favor manecillas 1
# Eje 3 : Palanaca Sensibilidad: Frente -1, Atras 1

# Informacion del tipo para los eventos generados
# JOYAXISMOTION = 7
# JOYBALLMOTION = 8
# JOYHATMOTION = 9
# JOYBUTTONDOWN = 10
# JOYBUTTONUP = 11



def node_joystick_traction():
    global joystick_ref, axis_moved, axis0, axis1, axis2, axis3, deadzone, arduino
    # Se inicializa el nodo de deteccion del joystick fisico llamado node_joystick_traction
    rospy.init_node('node_joystick_traction', anonymous=True)
    # Publica en el topico pwm_data
    pub_traction_orders = rospy.Publisher("Robocol/MotionControl/pwm_data", Float32MultiArray, queue_size=10)
    #pub_traction_orders = rospy.Publisher("/robocol/fpga/rpm", Int32MultiArray, queue_size=10)
    # Se define la tasa a la cual se ejecuta el nodo
    rate = rospy.Rate(10)

    # Iniciliza modulos de pygame necesarios para el llamado de eventos
    pygame.init()
    # Inicilizar el modulo pygame la deteccion del joystick
    pygame.joystick.init()

    # Referencia a envio de mensaje
    order = [0,0]
    #[Lado izquierdo, lado derecho, 6 (Adelante, atras, neutro, freno)]


    pub_order = Float32MultiArray()

    print('Esperando...')
    # Espera a que se conecte el joystick fisico al computador
    #while pygame.joystick.get_count() != 1 and not rospy.is_shutdown():
        #rate.sleep()
    print('Conectado')


    # Se crea la referencia al joystick
    joystick_ref = pygame.joystick.Joystick(0)
    # Se inicializa el joystick de esa referencia
    joystick_ref.init()
    # Referencia de la palanca estado actual
    ref_try_axis_3 = joystick_ref.get_axis(3)
    # Esperar a que se mueva palanca aceleracion para que se calibre
    while ref_try_axis_3 == joystick_ref.get_axis(3): #and not rospy.is_shutdown():
        #rate.sleep()
        pygame.event.clear()
        print('Mover palanca')
        pass  # Se corre el nodo hasta que este se finalice
    

    left,right = 0,0
    while not rospy.is_shutdown():
        empty_event_queue()
        
        if axis_moved:

            axis0 = joystick_ref.get_axis(0)
            axis1 = joystick_ref.get_axis(1)
            axis2 = joystick_ref.get_axis(2)
            axis3 = joystick_ref.get_axis(3)
            
            if abs(round(axis2,1)) <= deadzone_rot:
                left, right = stick_steering(axis1, axis0, max_rpm*(-axis3+1)/2)
            else:
                left, right = int((max_rpm*(-axis3+1)/2)*axis2), int(-(max_rpm*(-axis3+1)/2)*axis2)

            time.sleep(0.5)
            order[0], order[1] = left, right
            #order[2], order[3] = np.abs(left), np.abs(right)
            #order[0], order[1] = left, right
            #order[2], order[3] = left, right
            #encoded = (str(order)+"\n").encode('utf-8')
            print(order)

            #time.sleep(1)
            #order.header.stamp = rospy.Time.now()
            #order.header.seq = order.pub_order.data = orderheader.seq + 1
            #order.sensibility = int(max_rpm*(-axis3+1)/2)
            pub_order.data = order
            
            pub_traction_orders.publish(pub_order)
            axis_moved = False

        order[0], order[1] = 0,0
        #order[2], order[3] = left, right
        #order.header.stamp = rospy.Time.now()
        #order.header.seq = order.header.seq + 1
        #order.sensibility = int(100)
        #pub_order.data = order
        #pub_traction_orders.publish(pub_order)
        axis_moved = False
        rate.sleep()


def stick_steering(x, y, sensibilidad_rcv):
    # Convierte a polar
    r = math.hypot(-x, -y)
    t = math.atan2(-y, -x)
    # Rota 45 grados
    t += math.pi / 4
    # Retorna a cartesianas
    left = r * math.cos(t)
    right = r * math.sin(t)
    # Reescala a nuevas coordenadas
    left = left * math.sqrt(2)
    right = right * math.sqrt(2)
    # clamp to -1/+1
    left = max(min(left, 1), -1)
    right = max(min(right, 1), -1)

    min_mov = deadzone_stick*math.sqrt(2)

    if(r<=min_mov):
        left,right = 0,0

    return int(sensibilidad_rcv*left), int(sensibilidad_rcv*right)

def empty_event_queue():
    global axis_moved
    for event in pygame.event.get():
        if event.type == pygame.JOYAXISMOTION:
            axis_moved = True
        elif event.type == pygame.JOYBALLMOTION:
            pass
        elif event.type == pygame.JOYHATMOTION:
            pass
        elif event.type == pygame.JOYBUTTONDOWN:
            pass
        elif event.type == pygame.JOYBUTTONUP:
            pass

if __name__ == '__main__':
    
    node_joystick_traction()

