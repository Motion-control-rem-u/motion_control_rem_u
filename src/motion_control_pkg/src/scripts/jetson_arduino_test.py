#!/usr/bin/python3
import serial

arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=10)

while True:
	led_on = input('Do you want to move? ')[0]
	if led_on in 'y':
		sendArray = [1,1,1,1,100,100,100,100]
		arduino.write(sendArray, 8)
	if led_on in 'n':
		sendArray = [0,0,0,0,0,0,0,0]
		arduino.write(sendArray, 8)		