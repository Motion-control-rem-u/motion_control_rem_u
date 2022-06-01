#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray, Bool, Int16MultiArray
import serial
import serial.tools.list_ports
import time

#OBTIENE PWM DE CONTROL AUTONOMO O DE MANUAL Y LO ESCRIBE POR SERIAL.
#Corre en la Jetson
#funciona con joystick _publisher.py y control_rem.py
#ROBOCOL 2021-2

puerto = ''

#Verificamos el puerto del arduino para hacer la conexion
ports = list(serial.tools.list_ports.comports())
for p in ports:
	print(p.description)
	if "ttyACM" in p.description:
		print("This is an Arduino!")
		puerto = p.description



def main():
    print('Starting Arduino Serial Comunication node... \n')
    topic = 'Robocol/Power/voltages'
    rospy.init_node('Power_sense', anonymous=True) # Inicia el nodo de potencia
    rate = rospy.Rate(10)
    pub = rospy.Publisher(topic, String,queue_size=10)  
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if arduino.in_waiting > 0:
            line = arduino.readline().decode('utf-8').rstrip()
            print(line)
            pub(line)
            time.sleep(1)  
            # rospy.loginfo(line)
            # respuesta.publish(line)

    encoded = ("hola").encode('utf-8')
    arduino.write(encoded)
    rate.sleep()


if __name__== '__main__':
    try:
        # Crea la conexion serial con el Arduino
        arduino = serial.Serial('/dev/'+str(puerto), 115200, timeout=10)
        
        arduino.reset_input_buffer()
        main()
    except rospy.ROSInterruptException:
        print('Nodo detenido')