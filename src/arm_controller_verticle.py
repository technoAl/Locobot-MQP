import rospy
from pyrobot import Robot
import numpy as np
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Bool, Float32
from tf2_msgs.msg import TFMessage

class ArmController:

    def __init__(self):
        rospy.Subscriber('arm/go_to', Pose, self.goTo)
        rospy.Subscriber('arm/grip', Bool, self.gripperUpdate)
        rospy.Subscriber('arm/pickplace', PoseArray, self.pickplace)
	rospy.Subscriber('arm/pickhold', PoseArray, self.pickhold)
	rospy.Subscriber('arm/pour', Float32, self.pour)
	rospy.Subscriber('arm/pour_toggle', Bool, self.toggle)
	rospy.Subscriber('objects', TFMessage, self.update_cup_positions)
	rospy.Subscriber('pickup_blue_cup', Bool, self.pickup_blue_cup)
	rospy.Subscriber('arm/go_home', Bool, self.home)
        rospy.init_node('arm_controller', anonymous=True)
        self.robot = Robot('locobot')
        self.offset = [0.085, 0.0, 0.0]
	self.pitch = 0
	self.is_spin = False
	self.position = []
	self.blue_cup_trans = []
	self.blue_cup_rot = []
	self.white_cup_trans = []
	self.white_cup_rot = []
	self.arm_origin = []
        rospy.sleep(1)

    def update_cup_positions(self, msg):
	transforms = msg.transforms
	for transform in transforms:
	    if transform.child_frame_id == "white_cup_center":
		rotation = transform.transform.rotation
		self.white_cup_rot = euler_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w])
	        
		self.white_cup_trans = transform.transform.translation
	    if transform.child_frame_id == "blue_cup_handle":
		rotation = transform.transform.rotation
		self.blue_cup_rot = euler_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w])
		self.blue_cup_trans = transform.transform.translation
	    if transform.child_frame_id == "arm_origin":
		self.arm_origin = transform.transform.translation


    def adjust_cup(self, cup_pose):
	new_pose = [cup_pose[1], cup_pose[0], cup_pose[2]]
	pose = {"position": np.array([0.3, 0, 0.18]), "pitch": 1.57, "roll": 0, "numerical":False, "plan":False}
	self.robot.arm.set_ee_pose_pitch_roll(**pose)
	rospy.sleep(10.5)	
	pose = {"position": np.array([0.3, 0, 0.18]), "pitch": 0, "roll": 0, "numerical":False, "plan":False}
	self.robot.arm.set_ee_pose_pitch_roll(**pose)#pose = {"position": np.array([new_pose[0], new_pose[1], 0.18]), "pitch": 1.57, "roll": 0, "numerical":False, "plan":False}
	#self.robot.arm.set_ee_pose_pitch_roll(**pose)
	#rospy.sleep(0.1)
	#self.robot.gripper.close()
	#rospy.sleep(0.1)
	#pose = {"position": np.array([new_pose[0]+0.10, new_pose[1], 0.18]), "pitch": 1.57, "roll": 0, "numerical":False, "plan":False}
	#self.robot.arm.set_ee_pose_pitch_roll(**pose)
	#rospy.sleep(0.1)
	#self.robot.gripper.open()
	#return [new_pose[1], new_pose[0] + 0.10, 0.04]

    def go_to_vertical(self, pos, roll):
	pose = {"position": np.array([pos[0], pos[1], pos[2] + 0.10]), "pitch": 1.57, "roll": roll, "numerical":False, "plan": False}
	self.robot.arm.set_ee_pose_pitch_roll(**pose)
	rospy.sleep(0.3)

    def blue_cup_pickup_part(self, data, yaw):
	self.go_to_vertical(np.array([data[0], data[1], data[2] + 0.10]), 0)
	yaw = self.robot.arm.get_joint_angle('joint_1') - yaw

	self.go_to_vertical(np.array([data[0], data[1], data[2] + 0.10]), yaw)

	self.go_to_vertical(data, yaw)

    def go_to_horizontal(self, pos):
	pose = {"position": np.array([pos[0]-0.10, pos[1], pos[2]]), "pitch": 0, "roll": 0, "numerical":False, "plan": False}
	self.robot.arm.set_ee_pose_pitch_roll(**pose)
	rospy.sleep(0.3)
	 	
    def pickup_blue_cup(self, msg):
	if msg:
	   self.robot.gripper.open()
	   object_pose = [self.blue_cup_trans.x, self.blue_cup_trans.y, self.blue_cup_trans.z]
	   rospy.loginfo("X: " + str(object_pose[0]) + " Y: " + str(object_pose[1]) + " Z: " + str(object_pose[2]))
	   
	   rospy.loginfo("Roll: " + str(self.blue_cup_rot[0]) + " Pitch: " + str(self.blue_cup_rot[1]) + " Yaw: " + str(self.blue_cup_rot[2]))
	   self.blue_cup_pickup_part(object_pose, self.blue_cup_rot[2])
	   self.robot.gripper.close()
	   self.go_to_vertical([0.3, 0, 0.25],0)
	  
	   self.go_to_vertical([0.48,0,0.08], 0)
	   self.robot.gripper.open()
	   self.robot.arm.go_home()
	   self.go_to_horizontal([0.40, 0, 0.12])
	   self.go_to_horizontal([0.48, 0, 0.08])
	   self.robot.gripper.close()
	   self.go_to_horizontal([0.40, 0, 0.40])

	   self.go_to_horizontal([self.white_cup_trans.x, self.white_cup_trans.y, 0.4])	
			

    def home(self,msg):
	if msg:
	   self.robot.arm.go_home()
	   rospy.sleep(0.1)
	   self.robot.gripper.open()

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
        self.goTo_method(object_pose.position.x-0.05, object_pose.position.y, 0.4)
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
	

    def pickhold(self, msg):
	object_pose = msg.poses[0]
        target_pose = msg.poses[1]
        self.robot.arm.go_home()
        rospy.sleep(0.1)
        self.robot.gripper.open()
        self.goTo_method(object_pose.position.x-0.05, object_pose.position.y, 0.4)
        rospy.sleep(0.1)    
        self.goTo_method(object_pose.position.x, object_pose.position.y, object_pose.position.z)
        self.robot.gripper.close()
        rospy.sleep(0.1)    
        self.goTo_method(object_pose.position.x, object_pose.position.y, 0.4)
        rospy.sleep(0.1)
        self.goTo_method(target_pose.position.x, target_pose.position.y, 0.4)
        rospy.sleep(0.1)
        self.goTo_method(target_pose.position.x, target_pose.position.y, target_pose.position.z)
	

    def pour(self, msg):
	
	if self.is_spin:
	   self.pitch = msg.data
	   pose = {"position": np.array(self.position), "pitch": 0, "roll": self.pitch, "numerical":False, "plan":False}
	   self.robot.arm.set_ee_pose_pitch_roll(**pose)

    def pour_from_roll_continuous(self, msg):
	pose = {"position": np.array([0.40, 0, 0.40]), "pitch": 0, "roll": msg, "numerical":False, "plan":False}
	self.robot.arm.set_ee_pose_pitch_roll(**pose)

    def toggle(self, msg):
        if msg.data:
	   trans, rot, quat = self.robot.arm.pose_ee
	   self.position = np.array(trans)
	   self.is_spin = True
	else:
	   self.is_spin = False

    def run(self):
        self.robot.gripper.open()
        self.robot.arm.go_home()
        rospy.spin()

if __name__ == '__main__':
    try:
        ArmController().run()

    except rospy.ROSInterruptException:
        pass
