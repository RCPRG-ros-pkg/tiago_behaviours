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

ACK_WAIT_MAX_TIME_S = 30

#import pl_nouns.odmiana as ro
#dict_o = ro.OdmianaRzeczownikow()
#word_b = self.o.getBiernikLp(blocks, mianownik=word_m)
#if len(word_b) == 0:
#    word_b = self.o.getBiernikLm(blocks, mianownik=word_m)


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
    def __init__(self, is_simulated, conversation_interface):
        smach.State.__init__(self, input_keys=['goods_name'],
                             outcomes=['ok', 'preemption', 'timeout', 'error'])

        self.conversation_interface = conversation_interface
        self.rico_says_pub = rospy.Publisher('rico_says', String, queue_size=10)

    def execute(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))

        self.rico_says_pub.publish( 'Podaj mi {"' + str(userdata.goods_name.goods_name) + '", biernik} i potwierdz' );

        self.conversation_interface.addRequest('ack')
        self.conversation_interface.addRequest('ack_i_gave')

        start_time = rospy.Time.now()
        while True:
            end_time = rospy.Time.now()
            loop_time = end_time - start_time
            loop_time_s = loop_time.secs

            if loop_time_s > ACK_WAIT_MAX_TIME_S:
                self.conversation_interface.removeRequest('ack')
                self.conversation_interface.removeRequest('ack_i_gave')
                return 'timeout'

            if self.preempt_requested():
                self.service_preempt()
                self.conversation_interface.removeRequest('ack')
                self.conversation_interface.removeRequest('ack_i_gave')
                return 'preemption'

            if self.conversation_interface.has('ack'):
                self.conversation_interface.remove('ack')
                return 'ok'
            elif self.conversation_interface.has('ack_i_gave'):
                self.conversation_interface.remove('ack_i_gave')
                return 'ok'

            rospy.sleep(0.1)

        raise Exception('Unreachable code')

class SayTakeGoods(smach.State):
    def __init__(self, is_simulated, conversation_interface):
        smach.State.__init__(self, input_keys=['goods_name'],
                             outcomes=['ok', 'preemption', 'error'])

        self.conversation_interface = conversation_interface
        self.rico_says_pub = rospy.Publisher('rico_says', String, queue_size=10)

    def execute(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))

        self.rico_says_pub.publish( 'Odbierz {"' + str(userdata.goods_name.goods_name) + '", biernik} i potwierdz' );

        self.conversation_interface.addRequest('ack')
        self.conversation_interface.addRequest('ack_i_took')

        start_time = rospy.Time.now()
        while True:
            end_time = rospy.Time.now()
            loop_time = end_time - start_time
            loop_time_s = loop_time.secs

            if loop_time_s > ACK_WAIT_MAX_TIME_S:
                self.conversation_interface.removeRequest('ack')
                self.conversation_interface.removeRequest('ack_i_took')
                return 'timeout'

            if self.preempt_requested():
                self.service_preempt()
                self.conversation_interface.removeRequest('ack')
                self.conversation_interface.removeRequest('ack_i_took')
                return 'preemption'

            if self.conversation_interface.has('ack'):
                self.conversation_interface.remove('ack')
                return 'ok'
            elif self.conversation_interface.has('ack_i_took'):
                self.conversation_interface.remove('ack_i_took')
                return 'ok'

            rospy.sleep(0.1)

        raise Exception('Unreachable code')

class BringGoods(smach.StateMachine):
    def __init__(self, is_simulated, conversation_interface):
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
            smach.StateMachine.add('SetNavParams', navigation.SetNavParams(is_simulated),
                                    transitions={'ok':'MoveToKitchen', 'preemption':'PREEMPTED', 'error': 'FAILED'},
                                    remapping={'max_lin_vel_in':'max_lin_vel', 'max_lin_accel_in':'max_lin_accel'})

            smach.StateMachine.add('MoveToKitchen', navigation.MoveToComplex(is_simulated),
                                    transitions={'FINISHED':'AskForGoods', 'PREEMPTED':'PREEMPTED', 'FAILED': 'FAILED'},
                                    remapping={'nav_goal_pose':'kitchen_pose'})

            smach.StateMachine.add('AskForGoods', SayAskForGoods(is_simulated, conversation_interface),
                                    transitions={'ok':'MoveBack', 'preemption':'PREEMPTED', 'error':'FAILED', 'timeout':'AskForGoods'},
                                    remapping={'goods_name':'goods_name'})

            smach.StateMachine.add('MoveBack', navigation.MoveToComplex(is_simulated),
                                    transitions={'FINISHED':'SayGiveGoods', 'PREEMPTED':'PREEMPTED', 'FAILED': 'FAILED'},
                                    remapping={'nav_goal_pose':'initial_pose'})

            smach.StateMachine.add('SayGiveGoods', SayTakeGoods(is_simulated, conversation_interface),
                                    transitions={'ok':'FINISHED', 'preemption':'PREEMPTED', 'error': 'FAILED'},
                                    remapping={'goods_name':'goods_name'})
