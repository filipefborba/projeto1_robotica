#! /usr/bin/env python
# -*- coding:utf-8 -*-
__author__ = ["Rachel P. B. Moraes", "Fabio Miranda"]
import rospy
# import os
import numpy
from numpy import linalg
import transformations
from tf import TransformerROS
import tf2_ros
import math
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from kobuki_msgs.msg import BumperEvent
from neato_node.msg import Bump

# os.system('roslaunch tag_tracking ar_track_neato.launch') # opens camera tracking
id = 0
frame = "camera_frame" #mudar p/ head_camera ou camera_frame, para usb_cam ou robotcam respectivamente
tfl = 0
tf_buffer = tf2_ros.Buffer()
empty_counter = 0 # counter for the consecutive empty markers subscription
searchturn = "right"
lastcoords = [None]*3 # lastcoords[x,y,z] for action history
ls = 0.15 # LINEAR SPEED
issafe = True #Inital safery state
counter = 1 # Counter for scout mode
multiplier = 0.5 ### Poportional steering coefficient
rs = 4 ## Retreat speed in danger
historysize = 3 ## amount of actions that get cached in case of a missed marker
sp = 0.35 ## scout angular speed

## Actions ---------------------------------------------------------------------
def move_forward(x):
	if issafe:
		if x > 0.15:
			print 'Adjust trajectory to the right! abs dist:',round(math.fabs(0-multiplier*x),2)
			# print('modulo dist', math.fabs(0-0.2*x))
			return vel.publish(Twist(Vector3(ls,ls,0), Vector3(0,0,-math.fabs(0-multiplier*x))))

		elif x < -0.05:
			print 'Adjusting trajectory to the left! abs dist:',round(math.fabs(0-multiplier*x),2)
			# print('modulo dist',math.fabs(0-0.2*x))
			return vel.publish(Twist(Vector3(ls,ls,0), Vector3(0,0,math.fabs(0-multiplier*x))))
		else:
			print "Going straight!"
			return vel.publish(Twist(Vector3(ls,ls,0), Vector3(0.0,0,0.0)))
	# else:
	# 	print "Too close to keep going forward!"

def stop(x):
	return vel.publish(Twist(Vector3(0,0,0), Vector3(0.0,0,0.0)))

def spin(x):
	print 'Spinning!'
	return vel.publish(Twist(Vector3(0,0,0), Vector3(0.0,0,10)))

def stopnsearch(x): ## SCOUT MODE
	global counter
	global searchturn
	if counter <= 5: #side == "right" and
		searchturn = "right"
		print("Turning " + searchturn)
		vel.publish(Twist(Vector3(0,0.0,0), Vector3(0,0,-sp)))
		counter += 1
	else:
		vel.publish(Twist(Vector3(0.0,0,0), Vector3(0,0,sp)))
		searchturn = "left"
		print("Turning " + searchturn)
		counter += 1
		if counter == 11:
			counter = 0

def retreat(lz):
	if lz < 1.8:
		print("Too close! Retreating!")
		vel.publish(Twist(Vector3(-0.3,-0.3,0), Vector3(0,0,0)))
	else:
		print("Far enough, Stopping")
		vel.publish(Twist(Vector3(0,0,0), Vector3(0,0,0)))

## MAIN HANDLERS ---------------------------------------------------------------

def handlebumps(msg):
	global issafe
	if msg.state == 1 and issafe == True:
		issafe = False
		print("The robot hit something! Backing off and engaging scout mode!")
		vel.publish(Twist(Vector3(-rs,-rs,0), Vector3(0,0,0)))
		rospy.sleep(5)
		issafe = True
		empty_counter = 3
		return stopnsearch(None)

def handlebumps2(msg):
	global issafe
	if msg != None:
		if msg.leftFront == 1 or msg.leftSide == 1 or msg.rightFront == 1 or msg.rightSide == 1 and issafe == True:
			issafe = False
			print("The robot hit something! Backing off and engaging scout mode!")
			vel.publish(Twist(Vector3(-rs,-rs,0), Vector3(0,0,0)))
			rospy.sleep(1)
			issafe = True
			return stopnsearch(None)


def handlemarkers(msg):
	global empty_counter
	global x
	global y
	global z
	global id
	global last_action
	global lastcoords
	global lastarg
	global issafe

	# Action history dealing ---------------------------------------------------
	if msg.markers == []:
		# print "No marker counter:" + str(empty_counter)
		print '\n'
		empty_counter += 1
		stop(None)
		if empty_counter <= 2:
			print("Temporary lost markers! Re-doing last action!")
			try:
				last_action(lastarg)
			except NameError:
				pass
		else:
			print("No markers in sight! Engaging scout mode")
			stopnsearch(searchturn)
	#---------------------------------------------------------------------------
	# Getting attributes from markers
	for marker in msg.markers:
		empty_counter = 0
		## TRANSFORMACAO P/ RETORNAR AS CORDENADAS DO MARCADOR CORRETAMENTE
		id = marker.id
		marcador = "ar_marker_" + str(id)
		header = Header(frame_id=marcador)
		try:
			trans = tf_buffer.lookup_transform(frame, marcador, rospy.Time(0))
		except LookupException:
			pass

		x = round(trans.transform.translation.x,2)
		y = round(trans.transform.translation.y,2)
		z = round(trans.transform.translation.z,2)
		t = transformations.translation_matrix([x, y, z])
		# Encontra as rotacoes e cria uma matriz de rotacao a partir dos quaternions
		r = transformations.quaternion_matrix([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
		m = numpy.dot(r,t) # Criamos a matriz composta por translacoes e rotacoes
		z_marker = [0,0,1,0] # Sao 4 coordenadas porque e'  um vetor em coordenadas homogeneas
		v2 = numpy.dot(m, z_marker)
		v2_n = v2[0:-1] # Descartamos a ultima posicao
		n2 = v2_n/linalg.norm(v2_n) # Normalizamos o vetor
		x_robo = [1,0,0]
		cosa = numpy.dot(n2, x_robo) # Projecao do vetor normal ao marcador no x do robo
		angulo_marcador_robo = round(math.degrees(math.acos(cosa)),2)

		## SISTEMA ANTIGO -----------------------------------------------------
		# x = round(marker.pose.pose.position.x * 1e308 *100,2)
		# y = round(marker.pose.pose.position.y * 1e308 *100,2)
		# z = round(marker.pose.pose.position.z * 1e308 *1000000000,2)
		# -----------------------------------------------------------
		print(marker.id, "x:", x, " y:", y, " z:", z, "angulo:",angulo_marcador_robo)  #maker_pose
		lastcoords[0] = x
		lastcoords[1] = y
		lastcoords[2] = z
		print '\n'

		# Dealing with marker inputs -------------------------------------------
		if empty_counter < historysize and issafe == True: # Check for robot safety and action history
			if marker.id == 8:
				lastarg = lastcoords[2]
				retreat(lastcoords[2])
				last_action = retreat

			elif marker.id == 5:
				lastarg = lastcoords[0]
				move_forward(lastarg)
				last_action = move_forward

			elif marker.id == 100:
				spin(None)
				last_action = spin
				lastarg = None

			else:
				stop(None)
				last_action = stop

## ROS Block -------------------------------------------------------------------

if __name__=="__main__":
	rospy.init_node("jenny")
	recebedor = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, handlemarkers)
	bumper = rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, handlebumps)
	neatobumper = rospy.Subscriber("/bump",Bump,handlebumps2, queue_size=10)
	vel = rospy.Publisher("cmd_vel", Twist, queue_size=10)
	tfl = tf2_ros.TransformListener(tf_buffer)

	try:
		while not rospy.is_shutdown():
			a = None
			rospy.sleep(0.01)

	except rospy.ROSInterruptException:
		print ('programa encerrado')
		vel.publish(Twist(Vector3(0,0,0), Vector3(0,0,0)))

