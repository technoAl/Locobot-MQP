import rospy
from pyrobot import Robot
import numpy as np
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool


class ArmController:

    def __init__(self):
        rospy.Subscriber('arm/go_to', Pose, self.goTo)
        rospy.Subscriber('arm/grip', Bool, self.gripperUpdate)
        rospy.init_node('arm_controller', anonymous=True)
        self.robot = Robot('locobot')
	self.offset = [0.19, 0.0, 0.21]
	rospy.sleep(1)

    def goTo(self, msg):
	x = msg.position.x + self.offset[0]
	y = msg.position.y + self.offset[1]
	z = msg.position.z + self.offset[2]
	pose = {"position": np.array([x, y, z]), "pitch": 1.5, "roll": 0, "numerical": False}
        pitch = 0
	roll = 0
	self.robot.arm.set_ee_pose_pitch_roll(**pose)

    def gripperUpdate(self, msg):
        if msg.data:
            self.robot.gripper.close()
        else:
            self.robot.gripper.open()

        
    def run(self):
	self.robot.gripper.open()
        self.robot.arm.go_home()
	rospy.spin()

if __name__ == '__main__':
    try:
        ArmController().run()
    except rospy.ROSInterruptException:
        pass
