import rospy
from pyrobot import Robot
import numpy as np
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Bool


class ArmController:

    def __init__(self):
        rospy.Subscriber('arm/go_to', Pose, self.goTo)
        rospy.Subscriber('arm/grip', Bool, self.gripperUpdate)
        rospy.Subscriber('arm/pickplace', PoseArray, self.pickplace)
	rospy.Subscriber('arm/follow', Pose, self.follow)
	self.eePose = rospy.Publisher('arm/ee_pose', Pose);
        rospy.init_node('arm_controller', anonymous=True)
        self.robot = Robot('locobot')
        self.offset = [0.085, 0.0, 0.0]
        rospy.sleep(1)

    def follow(self, msg):
	pos = np.array([msg.position.x, msg.position.y, msg.position.z])
	self.robot.arm.set_ee_pose(pos, self.iq, plan=False, numerical=False)	

    def goTo(self, msg):
        x = msg.position.x + self.offset[0]
        y = msg.position.y + self.offset[1]
        z = msg.position.z + self.offset[2]
        pose = {"position": np.array([x, y, z]), "pitch": 0, "roll": 0, "numerical": False}
        #euler = quaternion_from_euler(msg.orientation)
        
        #pitch = 0
        #roll = 0
        self.robot.arm.set_ee_pose_pitch_roll(**pose)

    def goTo_method(self, x, y, z):
        x = x + self.offset[0]
        y = y + self.offset[1]
        z = z + self.offset[2]
        pose = {"position": np.array([x, y, z]), "pitch": 0, "roll": 0, "numerical": False}
        #euler = quaternion_from_euler(msg.orientation)
        
        #pitch = 0
        #roll = 0
        self.robot.arm.set_ee_pose_pitch_roll(**pose)

    def gripperUpdate(self, msg):
        if msg.data:
            self.robot.gripper.close()
        else:
            self.robot.gripper.open()

    def pickplace(self, msg):
        object_pose = msg.poses[0]
        target_pose = msg.poses[1]
        self.robot.arm.go_home()
        rospy.sleep(0.1)
        self.robot.gripper.open()
        self.goTo_method(object_pose.position.x, object_pose.position.y, 0.4)
        rospy.sleep(0.1)    
        self.goTo_method(object_pose.position.x, object_pose.position.y, object_pose.position.z)
        self.robot.gripper.close()
        rospy.sleep(0.1)    
        self.goTo_method(object_pose.position.x, object_pose.position.y, 0.4)
        rospy.sleep(0.1)
        self.goTo_method(target_pose.position.x, target_pose.position.y, 0.4)
        rospy.sleep(0.1)
        self.goTo_method(target_pose.position.x, target_pose.position.y, target_pose.position.z)
        self.robot.gripper.open()
        rospy.sleep(0.1)
        self.goTo_method(target_pose.position.x, target_pose.position.y, 0.4)

    def run(self):
	
        self.robot.gripper.open()
        self.robot.arm.go_home()
        print(self.robot.arm.pose_ee)
	self.iq = self.robot.arm.pose_ee[2]
	r= rospy.Rate(30);
	while not rospy.is_shutdown():
		pose_msg = Pose()
		ee = self.robot.arm.pose_ee[0]
		q = self.robot.arm.pose_ee[2]
		pose_msg.position.x = ee[0][0]
		pose_msg.position.y = ee[1][0]
		pose_msg.position.z = ee[2][0]
		pose_msg.orientation.x = q[0]
		pose_msg.orientation.y = q[1]
		pose_msg.orientation.z = q[2]
		pose_msg.orientation.w = q[3]
		self.eePose.publish(pose_msg)
		r.sleep()

if __name__ == '__main__':
    try:
        ArmController().run()
    except rospy.ROSInterruptException:
        pass
