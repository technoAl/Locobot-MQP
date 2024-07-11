import rospy
from pyrobot import Robot
import numpy as np
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Pose, PoseArray, Point
from std_msgs.msg import Bool, Float32, String
from tf2_msgs.msg import TFMessage
import math


class ArmSort:

	def __init__(self):
		rospy.Subscriber('arm/go_to', Pose, self.go_to)
		rospy.Subscriber('objects', TFMessage, self.update_positions)
		rospy.Subscriber('arm/pickup_prism', String, self.pickup_prism, queue_size=1)
		rospy.Subscriber('arm/pickup_adjust', Point, self.pickup_adjust, queue_size=1)
		rospy.Subscriber('arm/pickup_toggle', Bool, self.pickup_toggle)
		rospy.Subscriber('arm/go_home', Bool, self.table_home)
		#rospy.Subscriber('arm/insert', Bool, self.insert)
		rospy.Subscriber('arm/insert_toggle', Bool, self.insert_toggle)
		rospy.Subscriber('arm/insert_adjust', Point, self.insert_adjust, queue_size=1)
		#rospy.Subscriber('arm/grip', Bool, self.gripperUpdate)
		#rospy.Subscriber('arm/insert', Point, self.insert, queue_size=1)
		self.command_pub = rospy.Publisher('unity/commands', String, queue_size=1)
		rospy.init_node('arm_controller', anonymous =True)
		self.robot = Robot('locobot')
		self.offset = [0.085, 0.0, 0.0]
		self.red_prism_rot = []
		self.red_prism_trans = []
		self.blue_prism_rot = []
		self.blue_prism_trans = []
		self.green_prism_rot = []
		self.green_prism_trans = []

		self.red_insert = []
		self.yaw = 0

		self.toggle_pickup = False
		self.position = []
		self.adjusted_pos = []

		rospy.sleep(1)

	def update_positions(self, msg):
		transforms = msg.transforms
		for transform in transforms:
			if transform.child_frame_id == "red_prism":
				rotation = transform.transform.rotation
				self.red_prism_rot = euler_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w])
				self.red_prism_trans = transform.transform.translation
			if transform.child_frame_id == "prism_insert_red":
				self.red_insert = transform.transform.translation

	def pickup_prism_helper(self, data, yaw):
		self.go_to_vertical(np.array([data[0], data[1], 0.17]), 0)
		yaw = self.robot.arm.get_joint_angle('joint_1') - yaw

		self.yaw = yaw

		self.go_to_vertical(np.array([data[0], data[1], 0.17]), yaw)
		self.go_to_vertical(np.array([data[0], data[1], 0.14]), yaw)

	def pickup_prism(self, msg):
		if msg.data == "Red":
			self.toggle_pickup = False
			self.robot.gripper.open()
			object_pose = [self.red_prism_trans.x, self.red_prism_trans.y, self.red_prism_trans.z]
			self.yaw = self.red_prism_rot[2]
			self.pickup_prism_helper(object_pose, self.red_prism_rot[2] + 3.14)
			self.command_pub.publish("prism adjust")

	
	def pickup_toggle(self,msg):
		if msg.data:
			trans, rot, quat = self.robot.arm.pose_ee
			self.position = np.array(trans)
			self.toggle_pickup = True
		elif not msg.data and self.toggle_pickup:
			self.pickup_prism_part2()
			self.toggle_pickup  = False
		else:
			self.toggle_pickup = False


	def pickup_adjust(self, msg):
		if self.toggle_pickup:
			#trans, rot, quat = self.robot.arm.pose_ee
		#position = np.array(trans)
		
		#x = position[0]
		#y = position[1]
		#z = position[2]
			x1 = msg.z
			y1 = -msg.x
			#theta = math.atan2(y, x)
			
			#x -= (x1 * math.cos(theta) + y1 * math.sin(theta))], 0)

			#y -= (x1 * math.sin(theta) + y1 * math.cos(theta))
			#adjusted_pos = np.array([x ,y, z-0.1])
			#print(str(x) +" "  + str(y))
			
			self.adjusted_pos = [self.position[0] + x1, self.position[1] + y1, self.position[2] - 0.1]
			rospy.loginfo("x: " + str(msg.x) + " y: " + str(msg.y))
			self.go_to_vertical(self.adjusted_pos, self.yaw)

	
	def pickup_prism_part2(self):

		self.go_to_vertical(np.array([self.adjusted_pos[0], self.adjusted_pos[1], 0.075]), self.yaw)

		self.robot.gripper.close()

		self.go_to_vertical(np.array([self.adjusted_pos[0], self.adjusted_pos[1], 0.13]), self.yaw)

		self.insert()

	def go_to(self,msg):
		x = msg.position.x
		y = msg.position.y
		z = msg.position.z
		pose = {"position": np.array([x,y,z]), "pitch": 0, "roll": 0, "numerical": False, "plan": False}

		self.robot.arm.set_ee_pose_pitch_roll(**pose)
	
	def insert(self):
		# if msg.data:
		self.robot.arm.go_home()
		# self.go_to_vertical([self.red_insert.x - 0.15, self.red_insert.y, self.red_insert.z + 0.2], self.yaw)
		# self.go_to_pitch([self.red_insert.x - 0.15, self.red_insert.y, self.red_insert.z + 0.2], 0)
		self.go_to_pitch([self.red_insert.x - 0.2, self.red_insert.y, 0.28], 0)
		self.go_to_pitch([self.red_insert.x - 0.2, self.red_insert.y, 0.25], 0)
		self.go_to_pitch([self.red_insert.x - 0.20, self.red_insert.y, 0.24], 0)
		self.go_to_pitch([self.red_insert.x - 0.18, self.red_insert.y, 0.225],0)
		rospy.loginfo("WAITING TO INSERT")
		self.command_pub.publish("insert adjust")
	
	def insert_toggle(self, msg):
		if msg.data:
			trans, rot, quat = self.robot.arm.pose_ee
			self.position = np.array(trans)
			self.toggle_pickup = True
		elif not msg.data and self.toggle_pickup:
			self.go_to_pitch([self.red_insert.x - 0.1, self.red_insert.y, 0.18], 0)
			self.robot.gripper.open()
			self.robot.arm.go_home()
			self.toggle_pickup  = False
		else:
			self.toggle_pickup = False

	def insert_adjust(self, msg):
		if self.toggle_pickup:
			#trans, rot, quat = self.robot.arm.pose_ee
		#position = np.array(trans)
		
		#x = position[0]
		#y = position[1]
		#z = position[2]
			x1 = msg.z
			y1 = -msg.x
			#theta = math.atan2(y, x)
			
			#x -= (x1 * math.cos(theta) + y1 * math.sin(theta))], 0)

			#y -= (x1 * math.sin(theta) + y1 * math.cos(theta))
			#adjusted_pos = np.array([x ,y, z-0.1])
			#print(str(x) +" "  + str(y))
			
			self.adjusted_pos = [self.position[0] + x1, self.position[1] + y1, self.position[2] - 0.1]
			rospy.loginfo("x: " + str(msg.x) + " y: " + str(msg.y))
			self.go_to_pitch(self.adjusted_pos, 0)

	def go_to_vertical(self, pos, roll):
		pose = {"position": np.array([pos[0], pos[1], pos[2] + 0.10]), "pitch": 1.57, "roll": roll, "numerical":False, "plan": False}
		self.robot.arm.set_ee_pose_pitch_roll(**pose)
		rospy.sleep(0.3)

	def go_to_horizontal(self, pos):
		hyp = math.sqrt(pos[0] ** 2 + pos[1] ** 2)
		new_hyp = hyp - 0.10
		ratio = new_hyp / hyp
		pose = {"position": np.array([pos[0]*ratio, pos[1]*ratio, pos[2]]), "pitch": 0, "roll": 0, "numerical":False, "plan": False}
		self.robot.arm.set_ee_pose_pitch_roll(**pose)
		rospy.sleep(0.2)

	def go_to_pitch(self, pos, pitch):	
		pose = {"position": np.array([pos[0], pos[1], pos[2]]), "pitch": pitch, "roll": 0, "numerical":False, "plan": False}
		self.robot.arm.set_ee_pose_pitch_roll(**pose)
		rospy.sleep(0.2)

	def table_home(self, msg):
		if msg.data:
			self.robot.arm.go_home()
			self.go_to_horizontal([0.48, 0, 0.08])
			self.robot.gripper.open()

	def run(self):
		self.robot.gripper.open()
		self.robot.arm.go_home()
		rospy.spin()

if __name__ == '__main__':
	try:
		ArmSort().run()

	except rospy.ROSInterruptException:
		pass