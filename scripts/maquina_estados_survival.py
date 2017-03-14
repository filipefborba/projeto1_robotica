from math import radians
from random import randint
print
class Moving(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Not_yet','Crash', 'Near'])

    def execute(self, userdata):
        global speed_output
        rospy.loginfo('Executing state Moving')
        speed = Twist(Vector3(1, 0, 0), Vector3(0, 0, 0)) #Andar reto
        speed_output.publish(speed)

#Survival Instincts
        if sensor receber que robo está proximo de obstáculo:
            speed = Twist(Vector3(0, 0, 0), Vectetor3(0, 0, 0)) #Parar
            return  'Near'
        if bumper bater em algo: #rostopic info /bump
            speed = Twist(Vector3(0, 0, 0), Vectetor3(0, 0, 0)) #Parar
            return 'Crash'
        else:
            return 'Not_yet'

class Moving_Back(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Moved_back'])

    def execute(self, userdata):
        global speed_output
        speed = Twist(Vector3(-1, 0, 0), Vector3(0, 0, 0)) #Andar para trás reto
        speed_output.publish(speed)

class Turning90(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['90_Turned'])

    def execute(self, userdata):
        global speed_output
        speed = Twist(Vector3(0, 0, 0), Vector3(0, 0, (radians(90)))) #Girar 90 graus
        speed_output.publish(speed)

class TurningRandom(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['SortedTurned'])

    def execute(self, userdata):
        global speed_output
        speed = Twist(Vector3(0, 0, 0), Vector3(0, 0, (radians(randint(45, 315))))) #Girar 90 graus
        speed_output.publish(speed)


def main():
    global speed_output
    speed_output= rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    bumper = rospy.Subscriber('/bump', Twist, queue_size=1)
    laser = rospy.Publisher('/scan', Twist, queue_size=1)


    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['Finish'])

    # Open the container
    with sm:
        #Adicionando máquina de estados
        smach.StateMachine.add('MOVE', Moving(), transitions={'Not_yet':'', 'Crash':'', 'Bump':''})
        #Se MOVE retornar Not_yet, executa-se a máquina de estados X. 
        #Se retornar Crash, executa-se Y, e Z se retornar Bump.
        smach.StateMachine.add('TURN_90', Turning90(), transitions={'90_Turned':''})
        smach.StateMachine.add('TURN_RANDOM', TurningRandom(), transitions={'SortedTurned':''})
        smach.StateMachine.add('MOVE_BACK', TurningRandom(), transitions={'SortedTurned':''})



    # Execute SMACH plan
    outcome = sm.execute()
    #rospy.spin()


if __name__ == '__main__':
    main()