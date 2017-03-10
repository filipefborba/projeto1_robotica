#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

cv_image = None
media = []
centro = []
atraso = 1.5

def processa(dado):
	global media
	global centro
	frame = dado
	frame_r = frame[:,:,2]
	frame_g = frame[:,:,1]
	frame_rg = cv2.subtract(frame_r, frame_g)

	ret, frame_rg = cv2.threshold(frame_rg, 75, 255, cv2.THRESH_BINARY)
	bordas = cv2.morphologyEx(frame_rg, cv2.MORPH_GRADIENT, np.ones((3, 3)) )
	contornos, arvore = cv2.findContours(bordas, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)	

	# imagem de saida
	disp = cv2.cvtColor(frame_rg, cv2.COLOR_GRAY2BGR)
	print('contornos', len(contornos))
	for cnt in contornos:
		cnt = cnt.reshape(cnt.shape[0], 2)
		print('num pontos', cnt.shape[0])
		media_cnt = np.sum(cnt, axis=0)/cnt.shape[0]
		cv2.circle(disp, tuple(media_cnt), 10, (128, 128, 0))	
		cv2.drawContours(disp, [cnt], -1, (0, 0, 255), 3)

	cv2.imshow('caixa', disp)

	cv2.waitKey(1)
	print(media)
	return (media)

def recebe(imagem):
	global cv_image
	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime
	delay = (lag.secs+lag.nsecs/1000000000.0)
	if delay > atraso:
		return 
	print("DELAY", delay)
	try:
		antes = time.clock()
		cv_image = bridge.imgmsg_to_cv2(imagem, "bgr8")
		cv2.imshow("video", cv_image)
		cv2.waitKey(1)
		processa(cv_image)
		depois = time.clock()
		print ("TEMPO", depois-antes)
	except CvBridgeError as e:
		print(e)
	


if __name__=="__main__":

	rospy.init_node("cor")
	recebedor = rospy.Subscriber("/camera/image_raw", Image, recebe, queue_size=10, buff_size = 2**24)

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

	cv2.namedWindow("caixa")
	cv2.namedWindow("video")

	try:

		while not rospy.is_shutdown():
			#print (media)
			#print (centro)
			vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
			if len(media) != 0 and len(centro) != 0:
				dif_x = media[0]-centro[0]
				dif_y = media[1]-centro[1]
				if math.fabs(dif_x)<30: #and math.fabs(dif_y)<50:
					vel = Twist(Vector3(0.5,0,0), Vector3(0,0,0))
				else:
					if dif_x > 0:
						# Vira a direita
						vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.2))
					else:
						# Vira a esquerda
						vel = Twist(Vector3(0,0,0), Vector3(0,0,0.2))

			velocidade_saida.publish(vel)
			rospy.sleep(0.1)

	except rospy.ROSInterruptException:
	    print("Ocorreu uma exceção com o rospy")


