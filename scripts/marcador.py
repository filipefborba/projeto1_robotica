#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import numpy
from numpy import linalg
import transformations
from tf import TransformerROS
import tf2_ros
import math
import time
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Header

x = 0
y = 0
z = 0
id = 0
tfl = 0

buffer = tf2_ros.Buffer()


x_desejado = 0.12
#y_desejado = 0.10
z_desejado = 1.80

def recebe(msg):
	global x
	global y
	global z
	global id
	for marker in msg.markers:
		x = round(marker.pose.pose.position.x, 2)
		y = round(marker.pose.pose.position.y, 2)
		z = round(marker.pose.pose.position.z, 2)
		#print(x)
		id = marker.id
		#print(marker.pose.pose)
		if id == 100:
			print(buffer.can_transform("base_link", "ar_marker_100", rospy.Time(0)))
			header = Header(frame_id= "ar_marker_100")
			trans = buffer.lookup_transform("base_link", "ar_marker_100", rospy.Time(0))
			t = transformations.translation_matrix([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
			r = transformations.quaternion_matrix([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
			m = numpy.dot(r,t)
			v2 = numpy.dot(m,[0,0,1,0])
			v2_n = v2[0:-1]
			n2 = v2_n/linalg.norm(v2_n)
			cosa = numpy.dot(n2,[1,0,0])
			print("ang",math.degrees(math.acos(cosa)))
			#vstamped = Vector3Stamped(vector=v, header= header)
			#optimus = TransformerROS()
			#v3out = optimus.transformVector3("base_link",vstamped)
			#print(dir(trans))
			#print(trans.transform)
			#print(trans)
			#vd = buffer.transform(vstamped, "base_link", new_type = Vector3)
			#print(buffer.can_transform("base_link", "camera_frame", rospy.Time(0)))
			#print (vd)

if __name__=="__main__":
	global tfl
	global buffer
	rospy.init_node("marcador")
	recebedor = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, recebe)
	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
	tfl = tf2_ros.TransformListener(buffer)


	try:

		while not rospy.is_shutdown():
			if id == 250:
				print("Estou na área A!")
				print ("x: ",x)
				print ("x desejado: ",x_desejado)
				if x_desejado < x-0.3:
					print("Vá para direita")
					# vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.1))
					# velocidade_saida.publish(vel)
					# rospy.sleep(0.2)
					# vel = Twist(Vector3(0.5, 0, 0), Vector3(0, 0, 0))
				 # 	velocidade_saida.publish(vel)
				 # 	rospy.sleep(0.2)
					# vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.1))
					# velocidade_saida.publish(vel)
					# rospy.sleep(0.2)

				elif x-0.3 <= x_desejado and x_desejado >= x+0.3:
					print("X CERTO")
					# vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
		 		# 	velocidade_saida.publish(vel)
					# rospy.sleep(0.05)

				else:
					print("Vá para esquerda")
					# vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.1))
					# velocidade_saida.publish(vel)
					# rospy.sleep(0.2)
					# vel = Twist(Vector3(0.5, 0, 0), Vector3(0, 0, 0))
				 # 	velocidade_saida.publish(vel)
				 # 	rospy.sleep(0.2)
					# vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.1))
					# velocidade_saida.publish(vel)
					# rospy.sleep(0.2)

				# print ("y: ",y)
				# print ("y desejado: ",y_desejado)
				# if y_desejado < y:
				# 	print("Vá para cima")
				# elif y_desejado == y:
				# 	print("Y certo")
				# else:
				# 	print("Vá para baixo")

				# print ("z: ",z)
				# print ("z desejado: ",z_desejado)
				# if z_desejado < z-0.5:
				# 	print("Vá para frente")
				# 	vel = Twist(Vector3(0.5, 0, 0), Vector3(0, 0, 0))
				# 	velocidade_saida.publish(vel)
				# 	rospy.sleep(0.05)

				# elif z-0.5 <= z_desejado or z_desejado >= z+0.5:
		 	# 		print("Z CERTO")
		 	# 		vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
		 	# 		velocidade_saida.publish(vel)
				# 	rospy.sleep(0.05)

				# else:
				# 	print("Vá para trás")
				# 	vel = Twist(Vector3(-0.5, 0, 0), Vector3(0, 0, 0))
				# 	velocidade_saida.publish(vel)
				# 	rospy.sleep(0.05)

			else:
				print("Não encontrei o marcador 250")
				vel = Twist(Vector3(0,0,0), Vector3(0,0,0))

			velocidade_saida.publish(vel)
			rospy.sleep(0.05)

	except rospy.ROSInterruptException:
	    print("Ocorreu uma exceção com o rospy")


