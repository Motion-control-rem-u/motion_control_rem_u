#!/usr/bin/env python
import rospy
import cv2
import os 
from nav_msgs.msg import Odometry

# publicar imagenes
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

global pos_x, pos_y
global pos_x_total_zed2, pos_y_total_zed2

#GRAFICA EN TIEMPO REAL LA POSICION DEL ROBOT EN EL MAPA
#CAMBIAR LA DIRECCION DE LA IMAGEN (gridmap), SOLO CORRER DESPUES DE CORRER PLANEACION
#ROBOCOL

pos_x, pos_y = 0, 0

#Variables para acumular todas las posiciones
pos_x_total_zed2 = [0]
pos_y_total_zed2 = [0]

scriptDir = os.path.dirname(__file__)
ruta_img = scriptDir + "/Mapa_Ruta_Planeada.png" #Utiliza el mapa que sale al correr "planeacion_a.py"
gridmap = cv2.imread(ruta_img,0) 

height, width = gridmap.shape

gridmap = cv2.cvtColor(gridmap, cv2.COLOR_GRAY2RGB)

def position_callback(msg): #Me regresa la posicion del robot segun Zed2
	global pos_x, pos_y, pos_x_total_zed2, pos_y_total_zed2
	pos_x = msg.pose.pose.position.x
	pos_y = msg.pose.pose.position.y

def pixels (coord,height, width) :
	x_len = 30
	y_len = 40
	x_scale = float(x_len  * width**(-1))
	y_scale = float(y_len  * height**(-1))
	x_center = width / 2 - 14/x_scale
	y_center = height / 2 - 2.4/y_scale

	x =  round((coord[0] / x_scale) +  x_center) 
	y =  round((coord [1] / y_scale ) + y_center)   
	return [int(x),int(y)]

#Funcion principal grafica
def main():
	global pos_x, pos_y
	global img_pub
	rospy.init_node('control', anonymous=True) #Inicio nodo

	rate = rospy.Rate(10) #10hz
	rospy.Subscriber("zed2/odom", Odometry, position_callback, tcp_nodelay=True)
	img_pub = rospy.Publisher('Robocol/MotionControl/imagen', Image, queue_size=1)
	print('Graficando...')

	while not rospy.is_shutdown():
		#pos_x_new = int(pos_x*10 + width/2)
		#pos_y_new = int(pos_y*10 + height/2)

		poses_new = pixels((pos_x,pos_y), height, width)

		pos_x_total_zed2.append(poses_new[0])
		pos_y_total_zed2.append(poses_new[1])
		
		for i in range(len(pos_y_total_zed2)):
			image = cv2.circle(gridmap, (pos_x_total_zed2[i],pos_y_total_zed2[i]), radius=5, color=(0, 0, 255), thickness=-1) #color=(Blue,Green,Red)
			img_final = cv2.resize(image, (594, 802), interpolation=cv2.INTER_AREA) #(594,802)
			#cv2.namedWindow('Plot')
			#cv2.resizeWindow('Plot',height/2, width/2)
			# cv2.imshow('Plot',img_final)
			# cv2.waitKey(1)
			# publicar imagen
			bridge = CvBridge()
			try:
				image_message = bridge.cv2_to_imgmsg(img_final, encoding= 'passthrough')
				img_pub.publish(image_message)

			except CvBridgeError as e:
				print(e)
			
	print(' Cerrando plot...')
	scriptDir = os.path.dirname(__file__)
	ruta_img2 = scriptDir + "/mapa_con_pose_robot.png"
	cv2.imwrite(ruta_img2, img_final)
	print('  Imagen final guardada.')

if __name__ == '__main__':
	main()	


