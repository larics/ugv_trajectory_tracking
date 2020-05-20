#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, PoseStamped, Pose
from std_msgs.msg import Int16
import math

from trajectory_msgs.msg import MultiDOFJointTrajectory
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point


class ugvControl:
    def __init__(self, rate, parameters, sim):
        self.sim = sim;
        self.rate = rate;
        self.command = Twist()
        self.twist_ref = Twist()
        self.position_reference = Pose()
        self.startFlag = False
        
        self.control_type = parameters['initial_control_type']
        self.Kp_v = parameters['Kp_v']
        self.Kp_w = parameters['Kp_w']
        self.sensitivity = parameters['sensitivity']
        self.v_limit = parameters["v_limit"]
        self.w_limit = parameters["w_limit"]
        self.robot_name = parameters['robot_name']

        print('Controller initialized with params: ', parameters)

    def poseCallback(self, msg):
        if (self.sim):
            index = -1
            for i in range(len(msg.name)):
                if msg.name[i] == self.robot_name:
                    index = i
            if index == -1:
                print("No such robot in Gazebo, cannot get model state.")

            if not self.startFlag:
                self.position_reference.position.x = msg.pose[index].position.x
                self.position_reference.position.y = msg.pose[index].position.y
                self.startFlag = True

            self.current_pose = msg.pose[index]

        else:
            if not self.startFlag:
                self.position_reference.position.x = msg.pose.position.x
                self.position_reference.position.y = msg.pose.position.y
                self.position_reference.orientation.x = msg.pose.orientation.x
                self.position_reference.orientation.y = msg.pose.orientation.y
                self.position_reference.orientation.z = msg.pose.orientation.z
                self.position_reference.orientation.w = msg.pose.orientation.w
                self.startFlag = True

            self.current_pose = msg.pose

    def refCallback(self, msg):
        if len(msg.points) >= 1:
            if len(msg.points[0].transforms) >= 1:
                self.position_reference.position.x = msg.points[0].transforms[0].translation.x
                self.position_reference.position.y = msg.points[0].transforms[0].translation.y
                self.position_reference.orientation.x = msg.points[0].transforms[0].rotation.x
                self.position_reference.orientation.y = msg.points[0].transforms[0].rotation.y
                self.position_reference.orientation.z = msg.points[0].transforms[0].rotation.z
                self.position_reference.orientation.w = msg.points[0].transforms[0].rotation.w
        else:
            print("Warning! Check the trajectory msg!")
 
    def getCommand(self):
        feed_forward = Twist()

        if (self.control_type == 0):
            self.position_reference.position.x = self.current_pose.position.x
            self.position_reference.position.y = self.current_pose.position.y
            self.position_reference.orientation.x = self.current_pose.orientation.x
            self.position_reference.orientation.y = self.current_pose.orientation.y
            self.position_reference.orientation.z = self.current_pose.orientation.z
            self.position_reference.orientation.w = self.current_pose.orientation.w
            feed_forward = self.twist_ref

        #self.controller.compute_control_actions(self.current_pose, self.current_twist, self.i)
        self.compute_control_actions(self.current_pose, self.get_reference())

        self.command.linear.x = self.v_c_n() + feed_forward.linear.x
        self.command.angular.z = self.w_c_n() + feed_forward.angular.z
        #self.i+=1
        return self.command

    def compute_control_actions(self, pose, reference):
        quaternion_meas = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
            )

        quaternion_ref = (
            reference.orientation.x,
            reference.orientation.y,
            reference.orientation.z,
            reference.orientation.w
            )

        theta_measured = euler_from_quaternion(quaternion_meas)[2]

        dx = reference.position.x - pose.position.x
        dy = reference.position.y - pose.position.y
        d = math.sqrt(dx ** 2 + dy ** 2);
        if (self.control_type == 2):
            theta_ref = euler_from_quaternion(quaternion_ref)[2]
        else:
            theta_ref = math.atan2(dy, dx)

        e_theta = theta_ref - theta_measured;

        e_theta = math.atan2(math.sin(e_theta), math.cos(e_theta))

        if (d > self.sensitivity):
            self.v_ref = self.Kp_v * math.sqrt(dx ** 2 + dy ** 2)
            self.w_ref = self.Kp_w * e_theta
        else:
            self.v_ref = 0.0
            self.w_ref = 0.0

        if (self.v_ref > self.v_limit):
            self.v_ref = self.v_limit
        elif (self.v_ref < -self.v_limit):
            self.v_ref = -self.v_limit

        if (self.w_ref > self.w_limit):
            self.w_ref = self.w_limit
        elif (self.w_ref < -self.w_limit):
            self.w_ref = -self.w_limit

    def v_c_n(self):
        return self.v_ref

    def w_c_n(self):
        return self.w_ref


    def get_reference(self):
        return self.position_reference
        #position1 = Point()
        #position1.x = 0.05 * t + 0.01
        #position1.y = 0.05 * t + 0.01
        #return position1

    def getStartFlag(self):
        return self.startFlag

    def controlTypeCallback(self, msg):
        if (self.control_type == 0 and (msg.data == 1 or msg.data == 2)):
            self.position_reference.position.x = self.current_pose.position.x
            self.position_reference.position.y = self.current_pose.position.y
            self.position_reference.orientation.x = self.current_pose.orientation.x
            self.position_reference.orientation.y = self.current_pose.orientation.y
            self.position_reference.orientation.z = self.current_pose.orientation.z
            self.position_reference.orientation.w = self.current_pose.orientation.w

        self.control_type = msg.data

    def cmdVelRefCallback(self, msg):
        self.twist_ref = msg;


if __name__ == '__main__':
    rospy.init_node('ugv_control')

    controller_parameters = {'Kp_v': 0.9, 'Kp_w': 1.0, 'v_deadzone': 0.001, 'w_deadzone': 0.0, \
                             'v_limit': 0.0, 'w_limit': 0.0}

    rate = rospy.get_param('~rate', 10)
    SIMULATION = rospy.get_param('~sim', bool(False))
    controller_parameters['Kp_v'] = rospy.get_param('~Kp_v', 0.9)
    controller_parameters['Kp_w'] = rospy.get_param('~Kp_w', 1.0)
    controller_parameters['v_limit'] = rospy.get_param('~v_limit', 0.1)
    controller_parameters['w_limit'] = rospy.get_param('~w_limit', 0.1)
    controller_parameters['sensitivity'] = rospy.get_param('~sensitivity', 0.001)
    controller_parameters['initial_control_type'] = rospy.get_param('~initial_control_type', 0)
    controller_parameters['robot_name'] = rospy.get_param('~robot_name', "/")

    rospyRate = rospy.Rate(rate)

    ugv_control = ugvControl(rate, controller_parameters, SIMULATION)

    if SIMULATION:
        poseSub = rospy.Subscriber('gazebo/model_states', ModelStates, ugv_control.poseCallback)
    else:
        poseSub = rospy.Subscriber('pose', PoseStamped, ugv_control.poseCallback)
   
    referenceSub = rospy.Subscriber('reference', MultiDOFJointTrajectory, ugv_control.refCallback)
    cmdVelRefSub = rospy.Subscriber('cmd_vel_ref', Twist, ugv_control.cmdVelRefCallback)
    controlTypeSub = rospy.Subscriber('control_type', Int16, ugv_control.controlTypeCallback)
    # control_type = 0 -> cmd vel passthrough
    # control_type = 1 -> this node computes orientation
    # control_type = 2 -> orientation is taken from the provided trajectory ref
    commandPub = rospy.Publisher('computed_control_actions', Twist, queue_size=1)

    command = None

    while not ugv_control.getStartFlag() and not rospy.is_shutdown():
        print('Waiting for pose measurements.')
        rospy.sleep(0.5)

    print("Starting ugv control.")

    while not rospy.is_shutdown():

        command = ugv_control.getCommand()
        if not SIMULATION:
            command.angular.z = - command.angular.z

        commandPub.publish(command)

        rospyRate.sleep()
