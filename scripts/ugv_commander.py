#!/usr/bin/env python

import rospy
import copy

from sensor_msgs.msg import Joy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, Pose, Transform, PoseStamped
from std_msgs.msg import Int16
from trajectory_msgs.msg import MultiDOFJointTrajectory
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint

class ugvCommander:
	def __init__(self, carrot_v, carrot_w, sim, rate, trajectory_rate):
		self.sim = sim
		self.rate = rate
		self.trajectory_rate = trajectory_rate
		self.trajectory_length = 0
		self.commander_mode = 0
		self.trajectory_reference = MultiDOFJointTrajectory()
		self.current_pose_reference = Pose()
		self.startFlag = False
		self.new_carrot_value = False
		self.carrot_v = carrot_v;
		self.carrot_w = carrot_w;
		self.carrot_x = 0.0
		self.carrot_y = 0.0
		self.new_trajectory = False
		self.current_pose = Pose()
		self.current_twist = Twist()
		self.trajectory_index = 0
		self.pointreference = Pose()
		self.w = 0
		self.v = 0

		self.setMode(0)

	def getMode(self):
		return self.commander_mode

	def setMode(self, mode):
		self.commander_mode = mode

		if (self.commander_mode == 0):
			print("Entering MANUAL mode.")
			self.w = 0
			self.v = 0
		elif (self.commander_mode == 1):
			print("Entering CARROT mode.")
			self.current_pose_reference.position.x = self.current_pose.position.x
			self.current_pose_reference.position.y = self.current_pose.position.y
		elif (self.commander_mode == 2):
			print("Entering TRAJECTORY mode.")
			self.current_pose_reference.position.x = self.current_pose.position.x
			self.current_pose_reference.position.y = self.current_pose.position.y
			self.current_pose_reference.orientation.x = self.current_pose.orientation.x
			self.current_pose_reference.orientation.y = self.current_pose.orientation.y
			self.current_pose_reference.orientation.z = self.current_pose.orientation.z
			self.current_pose_reference.orientation.w = self.current_pose.orientation.w
			self.trajectory_reference = MultiDOFJointTrajectory()
		elif (self.commander_mode == 3):
			print("Entering POINT mode.")
			self.pointreference.position.x = self.current_pose.position.x
			self.pointreference.position.y = self.current_pose.position.y

	def poseCallback(self, msg):
		if not self.startFlag:
			self.startFlag = True

		if (self.sim):
			self.current_pose = msg.pose[-1]
			self.current_twist = msg.twist[-1]
		else:
			self.current_pose = msg.pose
			self.current_twist = Twist()

	def joyCallback(self, msg):
		if (self.getMode() != 0 and msg.buttons[5] == 1):
			self.setMode(0)
		elif (self.getMode() != 1 and msg.buttons[4] == 1):
			self.setMode(1)
		elif (self.getMode() != 2 and msg.buttons[8] == 1):
			self.setMode(2)
		elif (self.getMode() != 3 and msg.buttons[0] == 1):
			self.setMode(3)

		if (self.getMode() == 0):
			self.v = msg.axes[3]
			self.w = msg.axes[0]
		elif (self.getMode() == 1):
			self.new_carrot_value = True
			self.carrot_x = self.carrot_v * msg.axes[3]
			self.carrot_y = self.carrot_v * msg.axes[0]
			
	def getControllerMode(self):
		if (self.getMode() == 0):
			return 0
		elif (self.getMode() == 1 or self.getMode() == 2):
			return 1
		elif (self.getMode() == 3):
			return 1

	def getCommandTwist(self):
		command = Twist()
		command.linear.x = self.v
		command.angular.z = self.w

		return command

	def getStartFlag(self):
		return self.startFlag

	def getCarrotTrajectory(self):
		command = MultiDOFJointTrajectory()
		point = MultiDOFJointTrajectoryPoint()
		transform = Transform()
		twist = Twist()

		point.transforms.append(copy.deepcopy(transform))
		point.velocities.append(copy.deepcopy(twist))
		point.accelerations.append(copy.deepcopy(twist))
		command.points.append(copy.deepcopy(point))

		if (self.new_carrot_value):
			self.new_carrot_value = False
			self.current_pose_reference.position.x = self.current_pose.position.x + self.carrot_x
			self.current_pose_reference.position.y = self.current_pose.position.y + self.carrot_y

		command.points[0].transforms[0].translation.x = self.current_pose_reference.position.x
		command.points[0].transforms[0].translation.y = self.current_pose_reference.position.y

		return command

	def getCommandFromTrajectory(self):
		command = MultiDOFJointTrajectory()
		point = MultiDOFJointTrajectoryPoint()
		transform = Transform()
		twist = Twist()

		point.transforms.append(copy.deepcopy(transform))
		point.velocities.append(copy.deepcopy(twist))
		point.accelerations.append(copy.deepcopy(twist))
		command.points.append(copy.deepcopy(point))


		if (self.new_trajectory):
			if (self.trajectory_index < self.trajectory_length):
				self.current_pose_reference.position.x = self.trajectory_reference.points[self.trajectory_index].transforms[0].translation.x
				self.current_pose_reference.position.y = self.trajectory_reference.points[self.trajectory_index].transforms[0].translation.y
				self.current_pose_reference.orientation.x = self.trajectory_reference.points[self.trajectory_index].transforms[0].rotation.x
				self.current_pose_reference.orientation.y = self.trajectory_reference.points[self.trajectory_index].transforms[0].rotation.y
				self.current_pose_reference.orientation.z = self.trajectory_reference.points[self.trajectory_index].transforms[0].rotation.z
				self.current_pose_reference.orientation.w = self.trajectory_reference.points[self.trajectory_index].transforms[0].rotation.w
				self.trajectory_index += int(self.trajectory_rate/self.rate)
			else:
				self.new_trajectory = False
			
		command.points[0].transforms[0].translation.x = self.current_pose_reference.position.x
		command.points[0].transforms[0].translation.y = self.current_pose_reference.position.y
		command.points[0].transforms[0].rotation.x = self.current_pose_reference.orientation.x
		command.points[0].transforms[0].rotation.y = self.current_pose_reference.orientation.y
		command.points[0].transforms[0].rotation.z = self.current_pose_reference.orientation.z
		command.points[0].transforms[0].rotation.w = self.current_pose_reference.orientation.w

		return command

	def getPointReference(self):

		command = MultiDOFJointTrajectory()
		point = MultiDOFJointTrajectoryPoint()
		transform = Transform()
		twist = Twist()

		point.transforms.append(copy.deepcopy(transform))
		point.velocities.append(copy.deepcopy(twist))
		point.accelerations.append(copy.deepcopy(twist))
		command.points.append(copy.deepcopy(point))


		command.points[0].transforms[0].translation.x = self.pointreference.position.x
		command.points[0].transforms[0].translation.y = self.pointreference.position.y

		return command


	def trajectoryCallback(self, msg):
		if (self.getMode() == 2):
			self.new_trajectory = True
			self.trajectory_index = 0
			self.trajectory_length = len(msg.points)
			self.trajectory_reference = msg;

	def pointReferenceCallback(self, msg):
		self.pointreference = msg


if __name__ == '__main__':
	rospy.init_node('ugv_commander')

	rate = rospy.get_param('~rate', int(50))
	trajectory_sampling_rate = rospy.get_param('~trajectory_rate', int(100))
	carrot_w = rospy.get_param('~carrot_w', float(0.001))
	carrot_v = rospy.get_param('~carrot_v', float(0.05))
	SIMULATION = rospy.get_param('~sim', bool(False))

	commander = ugvCommander(carrot_v, carrot_w, SIMULATION, rate, trajectory_sampling_rate)
	controller_mode = Int16()

	if SIMULATION:
		poseSub = rospy.Subscriber('gazebo/model_states', ModelStates, commander.poseCallback)
	else:
		poseSub = rospy.Subscriber('pose', PoseStamped, commander.poseCallback)

	joySub = rospy.Subscriber('joy', Joy, commander.joyCallback)
	trajectoryRefSub = rospy.Subscriber('trajectory', MultiDOFJointTrajectory, commander.trajectoryCallback)
	pointRefSub = rospy.Subscriber('pose_ref', Pose, commander.pointReferenceCallback)
	
	cmd_vel_pub = rospy.Publisher('cmd_vel_ref', Twist, queue_size=1)
	modePub = rospy.Publisher('control_type', Int16, queue_size=1)
	trajectoryPub = rospy.Publisher('reference', MultiDOFJointTrajectory, queue_size=1)

	rospyRate = rospy.Rate(rate)

	while not commander.getStartFlag() and not rospy.is_shutdown():
		print('Waiting for pose measurements.')
		rospy.sleep(0.5)

	print('starting ugv commander.')

	while not rospy.is_shutdown():
		if (commander.getMode() == 0):
			#manual mode
			cmd_vel_pub.publish(commander.getCommandTwist())
		elif (commander.getMode() == 1):
			#carrot following mode
			trajectoryPub.publish(commander.getCarrotTrajectory())
		elif (commander.getMode() == 2):
			#external trajectory mode
			trajectoryPub.publish(commander.getCommandFromTrajectory())
		elif (commander.getMode() == 3):
			trajectoryPub.publish(commander.getPointReference())

		controller_mode.data = commander.getControllerMode()
		modePub.publish(controller_mode)

		rospyRate.sleep()