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
from rosplan_tiago_common.tiago_torso_controller import TiagoSpeechController, TiagoTorsoController, TiagoHeadController
from pal_common_msgs.msg import *
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose

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

class PickPose(smach.State):
    def __init__(self, is_simulated):
        smach.State.__init__(self, output_keys=['pose'],
                             outcomes=['ok', 'preemption', 'error'])

    def execute(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))

        places = ['kuchnia', 'warsztat', 'pokoj', 'sypialnia']
        place_name = random.choice( places )

        # TODO: get pose from the knowledge base
        if place_name == 'kuchnia':
            pose = makePose(3, 0.2, -math.pi/2)
        elif place_name in ['warsztat', 'warsztatu']:
            pose = makePose(1.55, 8.65, math.pi/2)
        elif place_name in ['pokoj', 'pok\\303\\263j']:
            pose = makePose(-0.15, -0.3, math.pi/2)
        elif place_name == 'sypialnia':
            pose = makePose(3, 5, math.pi/2)

        userdata.pose = pose

        return 'ok'

class Wander(smach.StateMachine):
    def __init__(self, is_simulated, conversation_interface):
        smach.StateMachine.__init__(self, outcomes=['PREEMPTED',
                                                    'FAILED',
                                                    'FINISHED'])
        self.userdata.max_lin_vel = 0.2
        self.userdata.max_lin_accel = 0.5

        with self:
            smach.StateMachine.add('SetNavParams', navigation.SetNavParams(is_simulated),
                                        transitions={'ok':'PickPose', 'preemption':'PREEMPTED', 'error': 'FAILED'},
                                        remapping={'max_lin_vel_in':'max_lin_vel', 'max_lin_accel_in':'max_lin_accel'})

            smach.StateMachine.add('PickPose', PickPose(is_simulated),
                                        transitions={'ok':'MoveTo', 'preemption':'PREEMPTED', 'error': 'FAILED'})

            smach.StateMachine.add('MoveTo', navigation.MoveToComplex(is_simulated, conversation_interface),
                                        transitions={'FINISHED':'PickPose', 'PREEMPTED':'PREEMPTED', 'FAILED': 'FAILED'},
                                        remapping={'nav_goal_pose':'pose'})
