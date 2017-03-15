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
        if marker.id == 100:
            header = Header(frame_id= "ar_marker_100")
            print("id:", marker.id)
            can_transf = buffer.can_transform("base_link", "ar_marker_100", rospy.Time(0))
            print("can:",can_transf)
            # if can_transf == False:
            #     sys.exit(0)
            # else:
            #     print("Can Transf = True")
            trans = buffer.lookup_transform("base_link", "ar_marker_100", rospy.Time(0))
            t = transformations.translation_matrix([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
            r = transformations.quaternion_matrix([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
            m = numpy.dot(r,t)
            v2 = numpy.dot(m,[0,0,1,0])
            v2_n = v2[0:-1]
            n2 = v2_n/linalg.norm(v2_n)
            cosa = numpy.dot(n2,[1,0,0])
            ang = math.degrees(math.acos(cosa))
            print ("angulo", ang)

def bumpers():
    pass

#Classe que faz o robô girar
class Spin(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['still_far','close_enough'])
    def execute(self, userdata):
        global speed
        rospy.loginfo('Executing state SPIN')
        if 1==1:
            #Esse Vector3(vel linear, x, vel angular); roda como o ciclo trigonométrico; + = sentido anti-horário, - = horário
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.2))
            speed.publish(vel)
            return 'still_far'
        else:
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.2))
            speed.publish(vel)
            return 'close_enough'

#Andar para frente
class MoveForward(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['move','crash'])
    def execute(self, userdata):
        global speed
        global id
        print(id)
        if 50 in id:
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
            speed.publish(vel)
            print("Mudei Velocidade")
            #vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.5))
            #speed.publish(vel)

            #andar para tras
            return 'crash' #Significa que ele bateu e parou
        else:
            #andar normalmente#
            #speed = Twist(Vector3(0, 0, 0), Vector3(0.5, 0, 0)) #Andar para frente
            return 'move' #Significa que ele andou
        id = []

#Classe que roda o programa inteiro quando executado no terminal
def main():
    global speed
    global buffer
    rospy.init_node('smach_example_state_machine') #Precisa disso para rodar!
    speed = rospy.Publisher('/cmd_vel', Twist, queue_size=1) #Velocidade do robô
    bumper = rospy.Subscriber('/bump', Twist, queue_size=1)
    laser = rospy.Publisher('/scan', Twist, queue_size=1)
    recebedor = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, procurando_marcador)


    #Cria a Máquina de Estados
    sm = smach.StateMachine(outcomes=['spin'])

    #Utilizando a máquina
    with sm:
        #Adicionando estados para a máquina: (Nome, Classe, transitions={}); transitions disso para isso {'disso' : 'isso'}
        smach.StateMachine.add('MOVEFORWARD', MoveForward(),
                               transitions={'crash':'SPIN',
                                            'move':'MOVEFORWARD'})

        smach.StateMachine.add('SPIN', Spin(),
                               transitions={'still_far':'SPIN',
                                            'close_enough':'SPIN'})


    #Executa as máquinas
    outcome = sm.execute()
    #rospy.spin()
    #while not rospy.is_shutdown():
    #    rospy.sleep(0.2)

#Apenas pra ver se está rodando o arquivo original
if __name__ == '__main__':
    main()
