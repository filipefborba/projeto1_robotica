from math import radians

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
        if bumper bater em algo:
            speed = Twist(Vector3(0, 0, 0), Vectetor3(0, 0, 0)) #Parar
            return 'Crash'
        else:
            return 'Not_yet'

class Turning(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Turned'])

    def execute(self, userdata):
        global speed_output
        speed = Twist(Vector3(0, 0, 0), Vector3(0, 0, (radians(90)))) #Girar 90 graus
        speed_output.publish(speed)




def main():
    global speed_output
    speed_output= rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    bumper = rospy.Publisher('/bump', Twist, queue_size=1)
    laser = rospy.Publisher('/scan', Twist, queue_size=1)


    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['Finish'])

    # Open the container
    with sm:
        #Adicionando máquina de estados
        smach.StateMachine.add('LONGE', Longe(), transitions={'ainda_longe':'ANDANDO', 'perto':'GIRANDO'})
        #LONGE, se retornar ainda_longe, executa ANDANDO. Se retornar perto, executa GIRANDO




    # Execute SMACH plan
    outcome = sm.execute()
    #rospy.spin()


if __name__ == '__main__':
    main()