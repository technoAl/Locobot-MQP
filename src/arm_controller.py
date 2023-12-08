mport rospy
from pyrobot import Robot
import numpy as np
from geometry_msgs.msg import Pose

class ArmController:

    def __init__(self):
        self.pub = rospy.Subscriber('arm/go_to', Pose, goTo)
        rospy.init_node('arm_controller', anonymous=True)
        self.robot = Robot('locobot')


    def goTo(self, msg):
        print(msg)
        
    def run(self):
        rospy.spin()
        self.robot.arm.go_home()

if __name__ == '__main__':
    try:
        ArmController().run()
    except rospy.ROSInterruptException:
        pass
