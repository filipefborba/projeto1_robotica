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
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Header
from neato_node.msg import Bump


#Aqui setamos as variáveis iniciais. O neato vai atualizando
x = 0
y = 0
z = 0
id = 0
ang = -500
has_bumped = False
laser_distance = 0

tfl = 0

tf_buffer = tf2_ros.Buffer()

#Quão perto queremos que ele fique dos objetos. Servem para os "if's"
x_desejado = 0.12
y_desejado = 0.10
z_desejado = 1.00

frame = "camera_frame"

#Função que procura o marcador
def procurando_marcador(msg):
    global x
    global y
    global z
    global ang
    global speed
    global id
    for marker in msg.markers:
        id = marker.id
        marcador = "ar_marker_" + str(id)

        print(tf_buffer.can_transform(frame, marcador, rospy.Time(0)))
        header = Header(frame_id=marcador)
        # Procura a transformacao em sistema de coordenadas entre a base do robo e o marcador numero 100
        # Note que para seu projeto 1 voce nao vai precisar de nada que tem abaixo, a
        # Nao ser que queira levar angulos em conta
        trans = tf_buffer.lookup_transform(frame, marcador, rospy.Time(0))

        # Separa as translacoes das rotacoes
        x = trans.transform.translation.x*100
        y = trans.transform.translation.y*100
        z = trans.transform.translation.z*100
        # Procura a transformacao em sistema de coordenadas entre a base do robo e o marcador numero 100
        # Note que para seu projeto 1 voce nao vai precisar de nada que tem abaixo, a
        # Nao ser que queira levar angulos em conta
        # Terminamos
        print("id: {} x {} y {} z {}".format(id, x,y,z))


#Função que detecta o funcionamento dos bumpers
def bumper_detection(bump):
    global has_bumped
    if bump.leftFront or bump.leftSide or bump.rightFront or bump.rightSide:
        has_bumped = True
    else:
        has_bumped = False

def laser_detection(laser):
    global laser_distance
    ranges = list(laser.ranges)
    print(len(ranges))
    for d in ranges[140:260]: #Anda pra trás
        if d < 0.5 and d != 0:
            laser_distance = 2
            break
    for d in ranges[0:140] + ranges[260:361]: #Anda pra frente
        if d < 0.25 and d != 0:
            laser_distance = 1
            break


#Classe que faz o robô girar
class Spin(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['not_found','found','crash', 'crash_back', 'following', '50', '100', 'finish'])
    def execute(self, userdata):
        global speed
        rospy.loginfo('Executing state SPIN')
        global id
        global x, y, z
        global laser_distance

        if has_bumped == True:
            #Vector3(linear, 0 ,0), Vector3(0,0,angular)
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
            speed.publish(vel)
            print("Stopped")
            id = 0
            return 'crash'

        if laser_distance == 1:
            id = 0
            laser_distance = 0
            return 'crash_back'

        if id == 50:
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 1))
            speed.publish(vel)
            rospy.sleep(6.25)
            print("Marcador 50")
            id = 0
            return '50'

        if id == 100:
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 1))
            speed.publish(vel)
            rospy.sleep(1)
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -1))
            speed.publish(vel)
            rospy.sleep(2)
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 1))
            speed.publish(vel)
            rospy.sleep(1)
            vel = Twist(Vector3(-1, 0, 0), Vector3(0, 0, 0))
            speed.publish(vel)
            rospy.sleep(1)
            vel = Twist(Vector3(1, 0, 0), Vector3(0, 0, 0))
            speed.publish(vel)
            rospy.sleep(1)
            print("Marcador 100")
            id = 0
            return '100'

        if id == 150:
            #Caso não funcione. coloque id != 0 e tire o if id == 150
            print(id)
            # if (x > -0.5) and (x < 0.12):
            if z > 40:
                if (x > -5) and (x < 5):
                    id = 0
                    return 'found'
                elif x < -5:
                    vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.1))
                    speed.publish(vel)
                    rospy.sleep(0.4)
                    id = 0
                    return 'following'
                elif x > 5:
                    vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.1))
                    speed.publish(vel)
                    rospy.sleep(0.4)
                    id = 0
                    return 'following'
            if z < 40:
                if (x > -1.5) and (x < 1):
                    id = 0
                    return 'found'
                elif x < -1.5:
                    vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.1))
                    speed.publish(vel)
                    rospy.sleep(0.4)
                    id = 0
                    return 'following'
                elif x > 1:
                    vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.1))
                    speed.publish(vel)
                    rospy.sleep(0.4)
                    id = 0
                    return 'following'

            if z < 15 and z > 0:
                vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
                speed.publish(vel)
                print("Stopped and FINISHED!")
                return 'finish'

            # if id == 150:
            #     print(id)
            #     vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.3))
            #     speed.publish(vel)
            #     rospy.sleep(0.4)
            #     return 'not_found'

        else:
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.1))
            speed.publish(vel)
            return 'not_found'

class MoveForward(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['move', 'crash', 'following_marker', 'finish'])
    def execute(self, userdata):
        global speed
        global has_bumped
        global laser_distance
        global id
        global x, y, z
        print(x,y,z)
        rospy.loginfo('Executing state MOVEFORWARD')

        if has_bumped == True: #Se dista 20cm de algo sólido, para pra não bater.
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
            speed.publish(vel)
            print("Stopped")
            return 'crash' #Significa que ele bateu e parou
            id = 0

        if id == 150 and z > 20:
            vel = Twist(Vector3(0.15, 0, 0), Vector3(0, 0, 0))
            speed.publish(vel)
            rospy.sleep(0.5)
            return 'following_marker' #Segue o marcador

        if z < 15 and z > 0:
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
            speed.publish(vel)
            print("Stopped and FINISHED!")
            return 'finish'

        else: #Nenhum está acionado porque não entrou no 'if' de cima
            vel = Twist(Vector3(0.15, 0, 0), Vector3(0, 0, 0)) #Andar para frente
            speed.publish(vel)
            rospy.sleep(0.3)
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
        global id
        rospy.loginfo('Executing state TURNINGRANDOM')
        vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.5)) #Girar 90 graus
        speed.publish(vel)
        rospy.sleep(random.randint(5,25)/10)
        return 'sortedturn'

#Classe que roda o programa inteiro quando executado no termina1
def main():
    global speed
    global buffer
    rospy.init_node('marcador') #Precisa disso para rodar!
    speed = rospy.Publisher('/cmd_vel', Twist, queue_size=1) #Velocidade do robô
    bumper = rospy.Subscriber('/bump', Bump, bumper_detection)
    laser = rospy.Subscriber('/scan', LaserScan, laser_detection)
    recebedor = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, procurando_marcador)

    tfl = tf2_ros.TransformListener(tf_buffer)


    #Cria a Máquina de Estados
    sm = smach.StateMachine(outcomes=['finish'])

    #Utilizando a máquina
    with sm:
        #Adicionando estados para a máquina: (Nome, Classe, transitions={}); transitions disso para isso {'disso' : 'isso'}
        smach.StateMachine.add('SPIN', Spin(),
                               transitions={'not_found':'SPIN',
                                            'found':'MOVEFORWARD',
                                            'crash': 'MOVEBACK',
                                            'crash_back': 'MOVEFORWARD',
                                            'following':'MOVEFORWARD',
                                            '50': 'SPIN',
                                            '100': 'SPIN',
                                            'finish': 'finish'})
        smach.StateMachine.add('MOVEFORWARD', MoveForward(),
                               transitions={'crash':'MOVEBACK',
                               'move':'SPIN',
                               'following_marker':'SPIN',
                               'finish': 'finish'})

        smach.StateMachine.add('TURNINGRANDOM', TurningRandom(),
                               transitions={'sortedturn':'SPIN'})

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
