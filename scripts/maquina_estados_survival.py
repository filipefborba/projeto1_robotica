### Survival (1) - Bumper
class SurvivalBump(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Bump','Not_Bump'])

    def execute(self, userdata):
        if bumper bater em algo OR sensor receber que robo está proximo de obstáculo:
            speed = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)) #Parar
            andar para tras
            return 'Crash'
        else:
            return 'Not_Crash'


def main():
    speed = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    bumper = rospy.Publisher('/bump', Twist, queue_size=1)
    laser = rospy.Publisher('/scan', Twist, queue_size=1)


if __name__ == '__main__':
    main()
