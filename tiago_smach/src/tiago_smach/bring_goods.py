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
#from rosplan_tiago_scenarios_msgs.msg import GoWithAttendanceAction
#from rosplan_tiago_scenarios_msgs.msg import GoWithAttendanceActionGoal
#from rosplan_tiago_scenarios_msgs.msg import GoWithAttendanceActionFeedback
#from rosplan_tiago_scenarios_msgs.msg import GoWithAttendanceActionResult
#from rosplan_tiago_common.tiago_torso_controller import TiagoSpeechController, TiagoTorsoController, TiagoHeadController
from pal_common_msgs.msg import *
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose

from tiago_behaviours_msgs.msg import MoveToGoal

import navigation
import smach_rcprg

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

class SayAskForGoods(smach_rcprg.State):
    def __init__(self, is_simulated, conversation_interface):
        smach_rcprg.State.__init__(self, input_keys=['goods_name'],
                             outcomes=['ok', 'preemption', 'timeout', 'error', 'shutdown'])

        self.conversation_interface = conversation_interface

    def execute(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))

        self.conversation_interface.addSpeakSentence( 'Podaj mi {"' + str(userdata.goods_name.goods_name) + '", biernik} i potwierdz' )

        self.conversation_interface.addExpected('ack', True)
        self.conversation_interface.addExpected('ack_i_gave', True)
        self.conversation_interface.addExpected('q_current_task', False)

        start_time = rospy.Time.now()
        while True:
            end_time = rospy.Time.now()
            loop_time = end_time - start_time
            loop_time_s = loop_time.secs

            if self.__shutdown__:
                return 'shutdown'

            if loop_time_s > ACK_WAIT_MAX_TIME_S:
                self.conversation_interface.removeExpected('ack')
                self.conversation_interface.removeExpected('ack_i_gave')
                self.conversation_interface.removeExpected('q_current_task')
                return 'timeout'

            if self.preempt_requested():
                self.conversation_interface.removeExpected('ack')
                self.conversation_interface.removeExpected('ack_i_gave')
                self.conversation_interface.removeExpected('q_current_task')
                self.service_preempt()
                return 'preemption'

            if self.conversation_interface.consumeItem('q_current_task'):
                self.conversation_interface.addSpeakSentence( 'Czekam na polozenie {"' + str(userdata.goods_name.goods_name) + '", dopelniacz}' )

            if self.conversation_interface.consumeItem('ack') or\
                    self.conversation_interface.consumeItem('ack_i_gave'):
                self.conversation_interface.removeExpected('ack')
                self.conversation_interface.removeExpected('ack_i_gave')
                self.conversation_interface.removeExpected('q_current_task')
                return 'ok'

            rospy.sleep(0.1)

        raise Exception('Unreachable code')

class SayTakeGoods(smach_rcprg.State):
    def __init__(self, is_simulated, conversation_interface):
        smach_rcprg.State.__init__(self, input_keys=['goods_name'],
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])

        self.conversation_interface = conversation_interface

    def execute(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))

        self.conversation_interface.addSpeakSentence( 'Odbierz {"' + str(userdata.goods_name.goods_name) + '", biernik} i potwierdz' )

        self.conversation_interface.addExpected('ack', True)
        self.conversation_interface.addExpected('ack_i_took', True)
        self.conversation_interface.addExpected('q_current_task', False)

        start_time = rospy.Time.now()
        while True:
            end_time = rospy.Time.now()
            loop_time = end_time - start_time
            loop_time_s = loop_time.secs

            if self.__shutdown__:
                return 'shutdown'

            if loop_time_s > ACK_WAIT_MAX_TIME_S:
                self.conversation_interface.removeExpected('ack')
                self.conversation_interface.removeExpected('ack_i_took')
                self.conversation_interface.removeExpected('q_current_task')
                return 'timeout'

            if self.preempt_requested():
                self.conversation_interface.removeExpected('ack')
                self.conversation_interface.removeExpected('ack_i_took')
                self.conversation_interface.removeExpected('q_current_task')
                self.service_preempt()
                return 'preemption'

            if self.conversation_interface.consumeItem('q_current_task'):
                self.conversation_interface.addSpeakSentence( 'Czekam na odebranie {"' + str(userdata.goods_name.goods_name) + '", dopelniacz}' )

            if self.conversation_interface.consumeItem('ack') or\
                    self.conversation_interface.consumeItem('ack_i_took'):
                self.conversation_interface.removeExpected('ack')
                self.conversation_interface.removeExpected('ack_i_took')
                self.conversation_interface.removeExpected('q_current_task')
                return 'ok'

            rospy.sleep(0.1)

        raise Exception('Unreachable code')

class BringGoods(smach_rcprg.StateMachine):
    def __init__(self, is_simulated, conversation_interface):
        smach_rcprg.StateMachine.__init__(self, input_keys=['goods_name'],
                                        outcomes=['PREEMPTED',
                                                    'FAILED',
                                                    'FINISHED', 'shutdown'])
        self.userdata.max_lin_vel = 0.2
        self.userdata.max_lin_accel = 0.5

        # TODO: use knowledge base for this:

        self.userdata.kitchen_pose = MoveToGoal()
        self.userdata.kitchen_pose.pose = makePose(3, 0.2, -math.pi/2)
        self.userdata.kitchen_pose.pose_valid = True
        self.userdata.kitchen_pose.place_name = 'kuchnia'
        self.userdata.kitchen_pose.place_name_valid = True

        with self:
            smach_rcprg.StateMachine.add('RememberCurrentPose', navigation.RememberCurrentPose(is_simulated),
                                    transitions={'ok':'SetNavParams', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'current_pose':'initial_pose'})

            smach_rcprg.StateMachine.add('SetNavParams', navigation.SetNavParams(is_simulated),
                                    transitions={'ok':'MoveToKitchen', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'max_lin_vel_in':'max_lin_vel', 'max_lin_accel_in':'max_lin_accel'})

            smach_rcprg.StateMachine.add('MoveToKitchen', navigation.MoveToComplex(is_simulated, conversation_interface),
                                    transitions={'FINISHED':'AskForGoods', 'PREEMPTED':'PREEMPTED', 'FAILED': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'nav_goal_pose':'kitchen_pose'})

            smach_rcprg.StateMachine.add('AskForGoods', SayAskForGoods(is_simulated, conversation_interface),
                                    transitions={'ok':'MoveBack', 'preemption':'PREEMPTED', 'error':'FAILED',
                                    'timeout':'AskForGoods','shutdown':'shutdown'},
                                    remapping={'goods_name':'goods_name'})

            smach_rcprg.StateMachine.add('MoveBack', navigation.MoveToComplex(is_simulated, conversation_interface),
                                    transitions={'FINISHED':'SayGiveGoods', 'PREEMPTED':'PREEMPTED', 'FAILED': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'nav_goal_pose':'initial_pose'})

            smach_rcprg.StateMachine.add('SayGiveGoods', SayTakeGoods(is_simulated, conversation_interface),
                                    transitions={'ok':'FINISHED', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'goods_name':'goods_name'})
