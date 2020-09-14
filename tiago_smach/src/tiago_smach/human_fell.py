#!/usr/bin/env python
# encoding: utf8

import math
import random
import rospy
import smach
import smach_ros
import dynamic_reconfigure.client
import actionlib

from move_base_msgs.msg import *
from actionlib_msgs.msg import GoalStatus
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose

import navigation
import smach_rcprg

from pl_nouns.dictionary_client import DisctionaryServiceClient

import task_manager

ACK_WAIT_MAX_TIME_S = 30

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

class SetHumanAndDestination(smach_rcprg.TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface):
        smach_rcprg.TaskER.BlockingState.__init__(self, input_keys=['human_name'], output_keys=['human_pose', 'dest_pose'],
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])

        self.conversation_interface = conversation_interface

        self.description = u'Znajduję człowieka'

    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        #self.conversation_interface.addSpeakSentence( u'Zakończyłem zadanie' )
        self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe Ustalam gdzie jest człowiek' )
        userdata.human_pose = navigation.PoseDescription({'place_name':unicode(userdata.human_name)})

        if self.__shutdown__:
            return 'shutdown'
        return 'ok'

class CheckHumanState(smach_rcprg.TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface):
        smach_rcprg.TaskER.BlockingState.__init__(self, input_keys=['human_name'], output_keys=[],
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])

        self.conversation_interface = conversation_interface

        self.description = u'Sprawdzam stan człowieka'

    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        #self.conversation_interface.addSpeakSentence( u'Zakończyłem zadanie' )
        gender = ""
        if userdata.human_name in ["John", "Peter"]:
            gender = "powinien Pan"
        else:
            gender = "powinna Pani"
        if isinstance(userdata.human_name, str):
            human_name = userdata.human_name.decode('utf-8')
        human_name = human_name.encode('utf-8').decode('utf-8')


        self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe '+human_name+u', jak się czujesz?' )
        rospy.sleep(2)
        self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe Dziękuję za informację. Dowidzenia.x' )
        if self.__shutdown__:
            return 'shutdown'
        return 'ok'


class SayIFinished(smach_rcprg.TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface):
        smach_rcprg.TaskER.BlockingState.__init__(self,
                             outcomes=['ok', 'shutdown'])

        self.conversation_interface = conversation_interface

        self.description = u'Mówię, że zakończyłem'

    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        #self.conversation_interface.addSpeakSentence( u'Zakończyłem zadanie' )
        self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe zakończyłem zadanie' )

        if self.__shutdown__:
            return 'shutdown'
        return 'ok'

class HumanFell(smach_rcprg.StateMachine):
    def __init__(self, sim_mode, conversation_interface, kb_places):
        smach_rcprg.StateMachine.__init__(self, input_keys=['human_name','susp_data'], output_keys=['susp_data'],
                                        outcomes=['PREEMPTED',
                                                    'FAILED',
                                                    'FINISHED', 'shutdown'])
        self.userdata.max_lin_vel = 0.2
        self.userdata.max_lin_accel = 0.5
        # TODO: use knowledge base for this:
        self.userdata.default_height = 0.2
        self.userdata.lowest_height = 0.0

        self.description = u'Podaję rzecz'

        with self:
            smach_rcprg.StateMachine.add('SetHeightMid', navigation.SetHeight(sim_mode, conversation_interface),
                                    transitions={'ok':'SetHumanAndDestination', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'torso_height':'default_height'})

            smach_rcprg.StateMachine.add('SetHumanAndDestination', SetHumanAndDestination(sim_mode, conversation_interface),
                                    transitions={'ok':'SetNavParams', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={})

            smach_rcprg.StateMachine.add('SetNavParams', navigation.SetNavParams(sim_mode),
                                    transitions={'ok':'MoveToHuman', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'max_lin_vel_in':'max_lin_vel', 'max_lin_accel_in':'max_lin_accel'})

            smach_rcprg.StateMachine.add('MoveToHuman', navigation.MoveToComplex(sim_mode, conversation_interface, kb_places),
                                    transitions={'FINISHED':'CheckHumanState', 'PREEMPTED':'PREEMPTED', 'FAILED': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'goal':'human_pose', 'susp_data':'susp_data'})

            smach_rcprg.StateMachine.add('CheckHumanState', CheckHumanState(sim_mode, conversation_interface),
                                    transitions={'ok':'SayIFinished', 'preemption':'PREEMPTED', 'error':'FAILED'
                                    ,'shutdown':'shutdown', },
                                    remapping={'human_name':'human_name'})


            # smach_rcprg.StateMachine.add('SetHeightLow', navigation.SetHeight(sim_mode, conversation_interface),
            #                         transitions={'ok':'AskForGoods', 'preemption':'PREEMPTED', 'error': 'FAILED',
            #                         'shutdown':'shutdown'},
            #                         remapping={'torso_height':'lowest_height'})

            # smach_rcprg.StateMachine.add('AskForGoods', SayAskForGoods(sim_mode, conversation_interface),
            #                         transitions={'ok':'MoveToDestination', 'preemption':'PREEMPTED', 'error':'FAILED',
            #                         'timeout':'AskForGoods','shutdown':'shutdown', 'turn_around':'TurnAroundA1'},
            #                         remapping={'goods_name':'goal', 'q_load_answer_id':'q_load_answer_id'})

            # smach_rcprg.StateMachine.add('MoveBack', navigation.MoveToComplex(sim_mode, conversation_interface, kb_places),
            #                         transitions={'FINISHED':'SayTakeGoods', 'PREEMPTED':'PREEMPTED', 'FAILED': 'FAILED',
            #                         'shutdown':'shutdown'},
            #                         remapping={'goal':'initial_pose', 'susp_data':'susp_data'})

            # smach_rcprg.StateMachine.add('SayTakeGoods', SayTakeGoods(sim_mode, conversation_interface),
            #                         transitions={'ok':'SetHeightEnd', 'preemption':'PREEMPTED', 'error': 'FAILED',
            #                         'shutdown':'shutdown', 'timeout':'SayTakeGoods', 'turn_around':'TurnAroundB1'},
            #                         remapping={'goods_name':'goal', 'q_load_answer_id':'q_load_answer_id'})

            # smach_rcprg.StateMachine.add('TurnAroundA1', navigation.RememberCurrentPose(sim_mode),
            #                         transitions={'ok':'TurnAroundA2', 'preemption':'PREEMPTED', 'error': 'FAILED',
            #                         'shutdown':'shutdown'},
            #                         remapping={'current_pose':'current_pose'})

            # smach_rcprg.StateMachine.add('TurnAroundA2', navigation.TurnAround(sim_mode, conversation_interface),
            #                         transitions={'ok':'AskForGoods', 'preemption':'PREEMPTED', 'error': 'FAILED',
            #                         'shutdown':'shutdown', 'stall':'AskForGoods'},
            #                         remapping={'current_pose':'current_pose'})

            # smach_rcprg.StateMachine.add('TurnAroundB1', navigation.RememberCurrentPose(sim_mode),
            #                         transitions={'ok':'TurnAroundB2', 'preemption':'PREEMPTED', 'error': 'FAILED',
            #                         'shutdown':'shutdown'},
            #                         remapping={'current_pose':'current_pose'})

            # smach_rcprg.StateMachine.add('TurnAroundB2', navigation.TurnAround(sim_mode, conversation_interface),
            #                         transitions={'ok':'SayTakeGoods', 'preemption':'PREEMPTED', 'error': 'FAILED',
            #                         'shutdown':'shutdown', 'stall':'AskForGoods'},
            #                         remapping={'current_pose':'current_pose'})

            # smach_rcprg.StateMachine.add('SetHeightEnd', navigation.SetHeight(sim_mode, conversation_interface),
            #                         transitions={'ok':'SayIFinished', 'preemption':'PREEMPTED', 'error': 'FAILED',
            #                         'shutdown':'shutdown'},
            #                         remapping={'torso_height':'default_height'})

            smach_rcprg.StateMachine.add('SayIFinished', SayIFinished(sim_mode, conversation_interface),
                                    transitions={'ok':'FINISHED', 'shutdown':'shutdown'})
    def my_exe(self):
        pass
    # def execute(self, userdata):
    #     while not rospy.is_shutdown():
    #         print "GOODS: ", userdata.susp_data.getData()
    #         if userdata.susp_data.getData() == "suspension requirements from the task harmoniser":
    #             return 'SUSPENDED'
    #         rospy.sleep(1)
    #     print "END GOODS"
        # self.description = u'Podaję {"' + userdata.goal + u'", biernik}'
        # return super(BringGoods, self).execute(userdata)
