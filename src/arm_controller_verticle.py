import rospy
from pyrobot import Robot
import numpy as np
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Pose, PoseArray, Point
from std_msgs.msg import Bool, Float32, String
from tf2_msgs.msg import TFMessage
import math

class ArmController:

	def __init__(self):
		
    	rospy.Subscriber('arm/go_to', Pose, self.goTo)
    	rospy.Subscriber('arm/grip', Bool, self.gripperUpdate)
    	rospy.Subscriber('arm/pickplace', PoseArray, self.pickplace)
		rospy.Subscriber('arm/pickhold', PoseArray, self.pickhold)
		rospy.Subscriber('arm/pour', Float32, self.pour)
		rospy.Subscriber('arm/pour_toggle', Bool, self.toggle)
		rospy.Subscriber('objects', TFMessage, self.update_cup_positions)
		rospy.Subscriber('/arm/pickup/blue_cup', Bool, self.pickup_blue_cup)
		rospy.Subscriber('arm/go_home', Bool, self.table_home)
		rospy.Subscriber('arm/place_in_box', Bool, self.place_in_box)
		self.command_pub = rospy.Publisher('unity/commands', String, queue_size=1)
		rospy.Subscriber('arm/pour_adjust', Point, self.pour_adjust)
		rospy.Subscriber('arm/pickup_adjust', Point, self.pickup_adjust, queue_size=1)
		rospy.Subscriber('arm/pickup_toggle', Bool, self.pickup_toggle)
		rospy.init_node('arm_controller', anonymous =True)
    	self.robot = Robot('locobot')
        self.offset = [0.085, 0.0, 0.0]
		self.pitch = 0
		self.is_spin = False
		self.toggle_pickup = False
		self.position = []
		self.adjusted_pos = []
		self.blue_cup_trans = []
		self.blue_cup_rot = []
		self.white_cup_trans = []
		self.white_cup_rot = []
		self.box_trans = []
		self.box_rot = []
		self.yaw = 0
        rospy.sleep(1)
	# Update positions of workplace objects
    def update_cup_positions(self, msg):
	transforms = msg.transforms
	for transform in transforms:
	    if transform.child_frame_id == "white_cup_bowl":
		rotation = transform.transform.rotation
		self.white_cup_rot = euler_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w])
	        
		self.white_cup_trans = transform.transform.translation
	    if transform.child_frame_id == "blue_cup_handle":
		rotation = transform.transform.rotation
		self.blue_cup_rot = euler_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w])
		self.blue_cup_trans = transform.transform.translation
	    if transform.child_frame_id == "box_robo":
		rotation = transform.transform.rotation
		self.box_rot = euler_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w])
		self.box_trans = transform.transform.translation
		

	# Adjust grip on cup for pouring
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

	# Go to blue cup for pickup
    def blue_cup_pickup_part(self, data, yaw):
	self.go_to_vertical(np.array([data[0], data[1], data[2] + 0.10]), 0)
	yaw = self.robot.arm.get_joint_angle('joint_1') - yaw

	self.yaw = yaw

	self.go_to_vertical(np.array([data[0], data[1], data[2] + 0.10]), yaw)
	self.go_to_vertical(np.array([data[0], data[1], data[2] + 0.07]), yaw)
	#self.go_to_vertical(np.array([data[0], data[1], data[2] + 0.04]), yaw)

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
	 	
    def pickup_blue_cup(self, msg):
	if msg:
	   self.toggle_pickup = False
	   self.robot.gripper.open()
	   object_pose = [self.blue_cup_trans.x, self.blue_cup_trans.y, self.blue_cup_trans.z]
	   self.yaw = self.blue_cup_rot[2]
	   self.blue_cup_pickup_part(object_pose, self.blue_cup_rot[2])
	   self.command_pub.publish("arm hover")	
    
	# Updates gripper location based on adjusted positions from Unity, calls the next phase of the solution once release
    def pickup_toggle(self,msg):
	if msg.data:
	   trans, rot, quat = self.robot.arm.pose_ee
	   self.position = np.array(trans)
	   self.toggle_pickup = True
	elif not msg.data and self.toggle_pickup:
	   self.pickup_blue_cup_2()
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
	   
	   #x -= (x1 * math.cos(theta) + y1 * math.sin(theta))
	   #y -= (x1 * math.sin(theta) + y1 * math.cos(theta))
	   #adjusted_pos = np.array([x ,y, z-0.1])
	   #print(str(x) +" "  + str(y))
	  
	   self.adjusted_pos = [self.position[0] + x1, self.position[1] + y1, self.position[2] - 0.1]
 	   rospy.loginfo("x: " + str(msg.x) + " y: " + str(msg.y))
	   self.go_to_vertical(self.adjusted_pos, self.yaw)			
	   # Picks up blue cup after adjust, moves to location above white cup
    def pickup_blue_cup_2(self):


        self.go_to_vertical(np.array([self.adjusted_pos[0], self.adjusted_pos[1],self.adjusted_pos[2] - 0.03]), self.yaw)

        self.go_to_vertical(np.array([self.adjusted_pos[0], self.adjusted_pos[1],self.adjusted_pos[2] - 0.06]), self.yaw)


        self.robot.gripper.close()
        self.go_to_vertical([0.3, 0, 0.25],0)
	  
        self.go_to_vertical([0.48,0,0.08], 0)
        self.robot.gripper.open()
        self.robot.arm.go_home()
        self.go_to_pitch([0.40, 0, 0.40], -0.78)

        self.go_to_horizontal([0.40, 0, 0.12])
        self.go_to_horizontal([0.48, 0, 0.06])
        self.robot.gripper.close()
        self.go_to_horizontal([0.40, 0, 0.40])
	   
        self.go_to_pitch([self.white_cup_trans.x, self.white_cup_trans.y, 0.34], 0)
        self.go_to_pitch([self.white_cup_trans.x, self.white_cup_trans.y, 0.32], 0)
        self.go_to_pitch([self.white_cup_trans.x, self.white_cup_trans.y, 0.30], 0)
        self.go_to_pitch([self.white_cup_trans.x, self.white_cup_trans.y, 0.28], 0)
        self.go_to_pitch([self.white_cup_trans.x, self.white_cup_trans.y, 0.26], 0)
        self.go_to_pitch([self.white_cup_trans.x, self.white_cup_trans.y, 0.25], 0)	
        self.go_to_pitch([self.white_cup_trans.x, self.white_cup_trans.y, 0.24], 0)	
        self.command_pub.publish("blue_cup grabbed")

  
    def place_in_box(self, msg):
	if msg:
	    self.go_to_horizontal([0.40, 0, 0.40])
	    self.go_to_horizontal([0.48, 0, 0.08])
	    self.robot.gripper.open()
	    self.go_to_horizontal([0.40, 0, 0.12])
	    self.go_to_pitch([0.40, 0, 0.40], -0.78)
	    self.go_to_vertical([0.42, 0, 0.20], 0)
	    self.go_to_vertical([0.48,0,0.08], 0)
	    self.robot.gripper.close()
	    self.go_to_vertical([0.42, 0, 0.20], 0)

	    self.go_to_vertical([self.box_trans.x, self.box_trans.y, 0.3], self.box_rot[2]+1.57)
	    self.go_to_vertical([self.box_trans.x, self.box_trans.y, 0.10], self.box_rot[2]+1.57)
	    self.robot.gripper.open()
	    self.go_to_vertical([self.box_trans.x, self.box_trans.y, 0.3], self.box_rot[2]+1.57)
	    self.robot.arm.go_home()
		
	# Adjust function for the pour feature, updates gripper rotation
    def pour_adjust(self, msg):
	trans, rot, quat = self.robot.arm.pose_ee
	position = np.array(trans)
	   
	x = position[0]
	y = position[1]
	z = position[2]
	x1 = -msg.x
	y1 = -msg.z
	theta = math.atan2(y, x)
	print(str(x) + " " + str(y))
	x -= (x1 * math.cos(theta) + y1 * math.sin(theta))
	y -= (x1 * math.sin(theta) + y1 * math.cos(theta))
	print(str(x) +" "  + str(y))
	self.go_to_pitch([x, y, z], 0)	

    def table_home(self, msg):
	if msg:
	    self.robot.arm.go_home()
	    self.go_to_horizontal([0.48, 0, 0.08])
	    self.robot.gripper.open()

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
