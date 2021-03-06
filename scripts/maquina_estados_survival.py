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


#Variáveis iniciais
x = 0
y = 0
z = 0
id = 0
ang = -500
has_bumped = False
laser_distance = 0
tfl = 0
tf_buffer = tf2_ros.Buffer()
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
        #print(tf_buffer.can_transform(frame, marcador, rospy.Time(0)))
        header = Header(frame_id=marcador)
        trans = tf_buffer.lookup_transform(frame, marcador, rospy.Time(0))

        x = trans.transform.translation.x*100
        y = trans.transform.translation.y*100
        z = trans.transform.translation.z*100
        print("id: {} x {} y {} z {}".format(id, x,y,z))

def bumper_detection(bump):
    global has_bumped
    if bump.leftFront or bump.leftSide or bump.rightFront or bump.rightSide:
        has_bumped = True
    else:
        has_bumped = False

def laser_detection(laser):
    global laser_distance
    laser.angle_min = math.radians(-300) #Angulo que o laser comeca a varrer [radianos]
    laser.angle_max = math.radians(60)   #Angulo onde o laser para de varrer [radianos]
                                         #Teremos uma area de varredura de 120 graus, pegando a frente toda
    laser.range_min = 0.15  #Distancia minima para se considerar [metro]
    laser.range_min = 2     #Distancia máxima para se considerar [metro]

    if len(laser.ranges) != 0:
        if min(laser.ranges) != 0 and < 0.16:
            laser_distance = True #Se a distancia detectada for menor que 16 cm
        else:
            laser_distance = False
    else:
        laser_distance = False

#Classe que faz o robô girar
class Spin(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['not_found','found','crash','following'])
    def execute(self, userdata):
        global speed
        rospy.loginfo('Executing state SPIN')
        global id
        global x, y, z

        if has_bumped == True:
            #Vector3(linear, 0 ,0), Vector3(0,0,angular)
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
            speed.publish(vel)
            print("Stopped")
            id = 0
            return 'crash'

        if 1==2:
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
            speed.publish(vel)
            print("Stopped")
            id = 0
            return 'crash'

        if id != 0:
            print(id)
            # if (x > -0.5) and (x < 0.12):
            if z > 40:
                if (x > -10) and (x <10):
                    return 'found'
                elif x < -4:
                    vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.1))
                    speed.publish(vel)
                    id = 0
                    rospy.sleep(0.4)
                    return 'following'
                else:
                    vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.1))
                    speed.publish(vel)
                    id = 0
                    return 'following'
            if z < 40:
                if (x > -3) and (x < 3):
                    return 'found'
                elif x < -3:
                    vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.1))
                    speed.publish(vel)
                    id = 0
                    rospy.sleep(0.4)
                    return 'following'
                elif x > 3:
                    vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.1))
                    speed.publish(vel)
                    id = 0
                    return 'following'
        else:
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.1))
            speed.publish(vel)
            return 'not_found'

class MoveForward(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['move','crash', 'near_something', 'following_marker', 'finish'])
    def execute(self, userdata):
        global speed
        global has_bumped
        global laser_detection
        global id
        global x, y, z
        print(x,y,z)
        rospy.loginfo('Executing state MOVEFORWARD')

        if has_bumped == True: #Se dista 20cm de algo sólido, para pra não bater.
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
            speed.publish(vel)
            print("Stopped")
            return 'crash' #Significa que ele bateu e parou
        if 1==2:
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
            speed.publish(vel)
            print("Stopped")
            return 'near_something' #Significa que o laser detectou que o robô está muito perto de algum objeto sólido e para

        if id != 0:
            vel = Twist(Vector3(0.1, 0, 0), Vector3(0, 0, 0))
            speed.publish(vel)
            id = 0
            rospy.sleep(0.5)
            return 'following_marker' #Segue o marcador

        if z < 15 :
            return 'finish'

        else: #Nenhum está acionado porque não entrou no 'if' de cima
            vel = Twist(Vector3(0.1, 0, 0), Vector3(0, 0, 0)) #Andar para frente
            speed.publish(vel)
            rospy.sleep(0.5)
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
                                            'following':'MOVEFORWARD'})
        smach.StateMachine.add('MOVEFORWARD', MoveForward(),
                               transitions={'near_something': 'TURNINGRANDOM',
                               'crash':'MOVEBACK',
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
