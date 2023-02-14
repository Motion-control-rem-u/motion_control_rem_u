#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from pynput import keyboard

def node_keyboard_teleop():
    global order, pub_order, pub_traction_orders
    # Se inicializa el nodo de deteccion del joystick fisico llamado node_joystick_traction
    rospy.init_node('node_joystick_traction', anonymous=True)
    # Publica en el topico pwm_data
    pub_traction_orders = rospy.Publisher("Robocol/MotionControl/pwm_data", Float32MultiArray, queue_size=10)
        # Referencia a envio de mensaje
    order = [0,0,0,0]
    #[Lado izquierdo arriba, lado derecho arriba, Lado izquierdo abajo, lado derecho abajo, 6 (Adelante, atras, neutro, freno)]

    pub_order = Float32MultiArray()

    rate = rospy.Rate(10)

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()


    while not rospy.is_shutdown():
        
        rate.sleep()

def on_press(key):
    print('alphanumeric key {0} pressed'.format(key.char))
    if key.char =='w':
        order[0],order[1],order[2], order[3] = 100,100,100,100
        pass
    elif key.char == 'a':
        pass
    elif key.char == 's':
        order[0],order[1],order[2], order[3] = -100,-100,-100,-100
        pass
    elif key.char == 'd':
        pass
    order[0],order[1],order[2], order[3] = 0,0,0,0

    print(order)
    pub_order.data = order
    pub_traction_orders.publish(pub_order)

def on_release(key):
    print('{0} released'.format(key))
    order[0],order[1],order[2], order[3] = 0,0,0,0
    pub_order.data = order
    pub_traction_orders.publish(pub_order)

    pass
if __name__ == '__main__':
    try:
        node_keyboard_teleop()
    except rospy.ROSInterruptException:
        pass