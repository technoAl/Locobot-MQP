import rospy
from pyrobot import Robot
import numpy as np
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Bool

class ArmTest:

    def __init__(self):

        self.robot = Robot('locobot')

    def rotate(self):

        pose = {"position": np.array([0.28, 0.17, 0.22]), "roll": 0.5, "numerical": False}

        self.robot.arm.set_ee_pose_pitch_roll(pose)

    def run(self):
        self.rotate()
        rospy.spin()

if __name__ == '__main__':
    ArmTest.run()

