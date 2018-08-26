#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Int16
import math

from trajectory_msgs.msg import MultiDOFJointTrajectory
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point


class gvrBotControl:
    def __init__(self, rate, parameters, sim):
        self.sim = sim;
        self.rate = rate;
        self.command = Twist()
        self.twist_ref = Twist()
        self.position_reference = Point()
        self.startFlag = False
        self.control_type = 0

        self.Kp_v = parameters['Kp_v']
        self.Kp_w = parameters['Kp_w']
        self.sensitivity = parameters['sensitivity']
        self.v_limit = parameters["v_limit"]
        self.w_limit = parameters["w_limit"]

    def poseCallback(self, msg):
        if (self.sim):
            if not self.startFlag:
                self.position_reference.x = msg.pose[-1].position.x
                self.position_reference.y = msg.pose[-1].position.y
                self.startFlag = True

            self.current_pose = msg.pose[-1]
        else:
            if not self.startFlag:
                self.position_reference.x = msg.pose.position.x
                self.position_reference.y = msg.pose.position.y
                self.startFlag = True

            self.current_pose = msg.pose

    def refCallback(self, msg):
        if len(msg.points) >= 1:
            if len(msg.points[0].transforms) >= 1:
                self.position_reference.x = msg.points[0].transforms[0].translation.x
                self.position_reference.y = msg.points[0].transforms[0].translation.y
        else:
            print("Warning! Check the trajectory msg!")
 
    def getCommand(self):
        feed_forward = Twist()

        if (self.control_type == 0):
            self.position_reference.x = self.current_pose.position.x
            self.position_reference.y = self.current_pose.position.y
            feed_forward = self.twist_ref

        #self.controller.compute_control_actions(self.current_pose, self.current_twist, self.i)
        self.compute_control_actions(self.current_pose, self.get_reference())

        self.command.linear.x = self.v_c_n() + feed_forward.linear.x
        self.command.angular.z = self.w_c_n() + feed_forward.angular.z
        #self.i+=1
        return self.command

    def compute_control_actions(self, pose, reference):
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
            )
        theta_measured = euler_from_quaternion(quaternion)[2]

        dx = reference.x - pose.position.x
        dy = reference.y - pose.position.y
        d = math.sqrt(dx ** 2 + dy ** 2);
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

    def get_name(self):
        return str("gvrBotTrajectory")

    def controlTypeCallback(self, msg):
        if (self.control_type == 0 and msg.data == 1):
            self.position_reference.x = self.current_pose.position.x
            self.position_reference.y = self.current_pose.position.y

        self.control_type = msg.data

    def cmdVelRefCallback(self, msg):
        self.twist_ref = msg;


if __name__ == '__main__':
    rospy.init_node('gvr_bot_control')

    controller_parameters = {'Kp_v': 0.9, 'Kp_w': 1.0, 'v_deadzone': 0.001, 'w_deadzone': 0.0, \
                             'v_limit': 0.0, 'w_limit': 0.0}

    rate = rospy.get_param('~rate', 10)
    SIMULATION = rospy.get_param('~sim', bool(False))
    controller_parameters['Kp_v'] = rospy.get_param('~Kp_v', 0.9)
    controller_parameters['Kp_w'] = rospy.get_param('~Kp_w', 1.0)
    controller_parameters['v_limit'] = rospy.get_param('~v_limit', 0.1)
    controller_parameters['w_limit'] = rospy.get_param('~w_limit', 0.1)
    controller_parameters['sensitivity'] = rospy.get_param('~sensitivity', 0.001)

    rospyRate = rospy.Rate(rate)

    gvr_bot_control = gvrBotControl(rate, controller_parameters, SIMULATION)

    if SIMULATION:
        poseSub = rospy.Subscriber('gazebo/model_states', ModelStates, gvr_bot_control.poseCallback)
    else:
        poseSub = rospy.Subscriber('pose', PoseStamped, gvr_bot_control.poseCallback)
   
    referenceSub = rospy.Subscriber('reference', MultiDOFJointTrajectory, gvr_bot_control.refCallback)
    cmdVelRefSub = rospy.Subscriber('cmd_vel_ref', Twist, gvr_bot_control.cmdVelRefCallback)
    controlTypeSub = rospy.Subscriber('control_type', Int16, gvr_bot_control.controlTypeCallback)
    commandPub = rospy.Publisher('computed_control_actions', Twist, queue_size=1)

    command = None

    while not gvr_bot_control.getStartFlag() and not rospy.is_shutdown():
        print('Waiting for pose measurements.')
        rospy.sleep(0.5)

    print("Starting gvr bot control.")

    while not rospy.is_shutdown():

        command = gvr_bot_control.getCommand()
        if not SIMULATION:
            command.angular.z = - command.angular.z

        commandPub.publish(command)

        rospyRate.sleep()