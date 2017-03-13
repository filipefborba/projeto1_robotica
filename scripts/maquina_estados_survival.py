### Survival (1) - Bumper
class Moving(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Not_yet','Crash'])

    def execute(self, userdata):
        global speed_output
        rospy.loginfo('Executing state Moving')
        speed = Twist(Vector3(1, 0, 0), Vector3(0, 0, 0)) #Andar reto
        speed_output.publish(speed)

#Survival Instincts
        if bumper bater em algo OR sensor receber que robo está proximo de obstáculo:
            speed = Twist(Vector3(0, 0, 0), Vectetor3(0, 0, 0)) #Parar
            rospy.sleep(0.05) #Tempo de espera
            speed = Twist(Vector3(-1, 0, 0), Vectetor3(0, 0, 0))
            return 'Crash'
        else:
            return 'Not_yet'


def main():
    global speed_output
    speed = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    bumper = rospy.Publisher('/bump', Twist, queue_size=1)
    laser = rospy.Publisher('/scan', Twist, queue_size=1)


    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['terminei'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('LONGE', Longe(),
                               transitions={'ainda_longe':'ANDANDO',
                                            'perto':'GIRANDO'})
        smach.StateMachine.add('ANDANDO', Andando(),
                               transitions={'ainda_longe':'LONGE'})
        smach.StateMachine.add('GIRANDO', Girando(),
                                transitions={'alinhando': 'GIRANDO',
                                'alinhou':'LONGE2'})
        smach.StateMachine.add('LONGE2', Longe(),
                               transitions={'ainda_longe':'ANDANDO2',
                                            'perto':'terminei'})
        smach.StateMachine.add('ANDANDO2', Andando(),
                               transitions={'ainda_longe':'LONGE2'})


    # Execute SMACH plan
    outcome = sm.execute()
    #rospy.spin()


if __name__ == '__main__':
    main()