#!/usr/bin/env python

import math
import random
import rospy
import smach
import smach_ros
import dynamic_reconfigure.client
import actionlib

from move_base_msgs.msg import *
from actionlib_msgs.msg import GoalStatus
from rosplan_tiago_scenarios_msgs.msg import GoWithAttendanceAction
from rosplan_tiago_scenarios_msgs.msg import GoWithAttendanceActionGoal
from rosplan_tiago_scenarios_msgs.msg import GoWithAttendanceActionFeedback
from rosplan_tiago_scenarios_msgs.msg import GoWithAttendanceActionResult
from rosplan_tiago_common.tiago_torso_controller import TiagoSpeechController, TiagoTorsoController, TiagoHeadController
from pal_common_msgs.msg import *
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose
from std_msgs.msg import String

import navigation

def makePose(x, y, theta):
    q = quaternion_from_euler(0, 0, theta)
    result = Pose()
    result.position.x = x
    result.position.y = y
    result.orientation.x = q[0]
    result.orientation.y = q[1]
    result.orientation.z = q[2]
    result.orientation.w = q[3]
    return result

class SayAskForGoods(smach.State):
    def __init__(self, is_simulated):
        smach.State.__init__(self, input_keys=['goods_name'],
                             outcomes=['ok', 'preemption', 'error'])

        self.rico_says_pub = rospy.Publisher('rico_says', String, queue_size=10)

    def execute(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))

        self.rico_says_pub.publish( 'Poprosze ' + str(userdata.goods_name.goods_name) );
        return 'ok'

class SayTakeGoods(smach.State):
    def __init__(self, is_simulated):
        smach.State.__init__(self, input_keys=['goods_name'],
                             outcomes=['ok', 'preemption', 'error'])

        self.rico_says_pub = rospy.Publisher('rico_says', String, queue_size=10)

    def execute(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))

        self.rico_says_pub.publish( 'Wez sobie ' + str(userdata.goods_name.goods_name) );
        return 'ok'

class BringGoods(smach.StateMachine):
    def __init__(self, is_simulated):
        smach.StateMachine.__init__(self, input_keys=['goods_name'],
                                        outcomes=['PREEMPTED',
                                                    'FAILED',
                                                    'FINISHED'])
        self.userdata.max_lin_vel = 0.2
        self.userdata.max_lin_accel = 0.5

        # TODO: use knowledge base for this:
        self.userdata.kitchen_pose = makePose(3, 0.2, -math.pi/2)
        self.userdata.initial_pose = makePose(-0.15, -0.3, math.pi/2)   # pokoj, TODO: change to initial pose

        with self:
            smach.StateMachine.add('SetNavParams', navigation.SetNavParams(is_simulated), transitions={'ok':'MoveToKitchen', 'preemption':'PREEMPTED', 'error': 'FAILED'},
                                    remapping={'max_lin_vel_in':'max_lin_vel', 'max_lin_accel_in':'max_lin_accel'})

            smach.StateMachine.add('MoveToKitchen', navigation.MoveToComplex(is_simulated), transitions={'FINISHED':'AskForGoods', 'PREEMPTED':'PREEMPTED', 'FAILED': 'FAILED'},
                                    remapping={'nav_goal_pose':'kitchen_pose'})

            smach.StateMachine.add('AskForGoods', SayAskForGoods(is_simulated), transitions={'ok':'MoveBack', 'preemption':'PREEMPTED', 'error': 'FAILED'},
                                    remapping={'goods_name':'goods_name'})

            smach.StateMachine.add('MoveBack', navigation.MoveToComplex(is_simulated), transitions={'FINISHED':'SayGiveGoods', 'PREEMPTED':'PREEMPTED', 'FAILED': 'FAILED'},
                                    remapping={'nav_goal_pose':'initial_pose'})

            smach.StateMachine.add('SayGiveGoods', SayTakeGoods(is_simulated), transitions={'ok':'FINISHED', 'preemption':'PREEMPTED', 'error': 'FAILED'},
                                    remapping={'goods_name':'goods_name'})
