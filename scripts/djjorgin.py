#! /usr/bin/env python
# -*- coding:utf-8 -*-

#Vários imports...
import roslib
import rospy
import sys
import smach
import smach_ros
import rospy
import numpy
from numpy import linalg
import transformations
from tf import TransformerROS
import tf2_ros
import math
import random
import time
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from neato_node.msg import Bump

#Aqui setamos as variáveis iniciais. O neato vai atualizando
x = 0
y = 0
z = 100
id = []
ang = -500
has_bumped = False
laser_distance = 0

#Quão perto queremos que ele fique dos objetos. Servem para os "if's"
x_desejado = 0.12
y_desejado = 0.10
z_desejado = 1.00
tfl = 0

#Função que procura o marcador
def procurando_marcador(msg):
    global x
    global y
    global z
    global ang
    global speed
    global id
    for marker in msg.markers:
        x = round(marker.pose.pose.position.x, 2)
        y = round(marker.pose.pose.position.y, 2)
        z = round(marker.pose.pose.position.z, 2)
        id.append(marker.id)
        print(marker.id)


#Função que detecta o funcionamento dos bumpers
def bumper_detection(bump):
    global has_bumped
    if bump.leftFront or bump.leftSide or bump.rightFront or bump.rightSide:
        has_bumped = True
    else:
        has_bumped = False

#def laser_detection(laser_distance):

#Classe que faz o robô girar
class Spin(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['not_found','found','crash'])
    def execute(self, userdata):
        global speed
        rospy.loginfo('Executing state SPIN')
        if has_bumped == True:
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
            speed.publish(vel)
            print("Stopped")
            return 'crash'
        if x < x_desejado: #Se a posição do marcador estiver boa, roda no sentido contrário que estava girando.
            #Esse Vector3(vel linear, x, vel angular); roda como o ciclo trigonométrico; + = sentido anti-horário, - = horário
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.2))
            speed.publish(vel)
            return 'found'
        else:
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.2))
            speed.publish(vel)
            return 'not_found'

class MoveForward(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['move','crash', 'near_something'])
    def execute(self, userdata):
        global speed
        global has_bumped
        global id
        rospy.loginfo('Executing state MOVEFORWARD')
        if 50 in id:
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
            speed.publish(vel)
            print("Stopped")
            return 'near_something' #Significa que o laser detectou que o robô está muito perto de algum objeto sólido e para

        if has_bumped == True: #Se dista 20cm de algo sólido, para pra não bater.
                vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
                speed.publish(vel)
                print("Stopped")
                return 'crash' #Significa que ele bateu e parou
        else: #Nenhum está acionado porque não entrou no 'if' de cima
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)) #Andar para frente
            speed.publish(vel)
            return 'move' #Significa que nenhum bumper está acionado e ele anda

#Andar pra trás - sobrevivência
class MoveBack(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['moved_back'])

    def execute(self, userdata):
        global speed
        rospy.loginfo('Executing state MOVEBACK')
        vel = Twist(Vector3(-0.2, 0, 0), Vector3(0, 0, 0)) #Andar para trás reto
        speed.publish(vel)
        rospy.sleep(1)
        return 'moved_back'

#Gira 90 graus para algum lado
class TurningRandom(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['sortedturn'])

    def execute(self, userdata):
        global speed
        rospy.loginfo('Executing state TURNINGRANDOM')
        vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.5)) #Girar 90 graus
        speed.publish(vel)
        rospy.sleep(0.7)
        return 'sortedturn'

#Classe que roda o programa inteiro quando executado no termina1
def main():
    global speed
    global buffer
    rospy.init_node('smach_example_state_machine') #Precisa disso para rodar!
    speed = rospy.Publisher('/cmd_vel', Twist, queue_size=1) #Velocidade do robô
    bumper = rospy.Subscriber('/bump', Bump, bumper_detection)
    laser = rospy.Subscriber('/scan', Twist, queue_size=1)
    recebedor = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, procurando_marcador)


    #Cria a Máquina de Estados
    sm = smach.StateMachine(outcomes=['finish'])

    #Utilizando a máquina
    with sm:
        #Adicionando estados para a máquina: (Nome, Classe, transitions={}); transitions disso para isso {'disso' : 'isso'}
        smach.StateMachine.add('MOVEFORWARD', MoveForward(),
                               transitions={'near_something': 'MOVEBACK',
                               'crash':'MOVEBACK',
                               'move':'MOVEFORWARD'})

        smach.StateMachine.add('SPIN', Spin(),
                               transitions={'not_found':'SPIN',
                                            'found':'MOVEFORWARD',
                                            'crash': 'MOVEBACK'})
        smach.StateMachine.add('TURNINGRANDOM', TurningRandom(),
                               transitions={'sortedturn':'MOVEFORWARD'})

        smach.StateMachine.add('MOVEBACK', MoveBack(),
                       transitions={'moved_back':'TURNINGRANDOM'})
    #Executa as máquinas
    outcome = sm.execute()
    #rospy.spin()
    #while not rospy.is_shutdown():
    #    rospy.sleep(0.2)

#Apenas pra ver se está rodando o arquivo original
if __name__ == '__main__':
    main()

#Falta: achar bumper e laser scan. Quando ele nao acha por um tempo, ele dá uma andada e procura.
