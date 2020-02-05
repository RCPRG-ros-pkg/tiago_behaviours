#!/usr/bin/env python
# encoding: utf8

import threading

import rospy
import smach
import smach_ros

from std_msgs.msg import String
from tiago_msgs.msg import Command

import smach_rcprg

#
# The SM that govenrs the highest-level conversation.
#

class HearState(smach_rcprg.State):
    def __init__(self, conversation_interface):
        smach_rcprg.State.__init__(self, output_keys=[],
                                    outcomes=['preemption', 'should_speak', 'error', 'shutdown'])
        self.conversation_interface = conversation_interface
        self.__intent_list__ = set()
        self.__intent_list_lock__ = threading.Lock()

        self.sub = rospy.Subscriber("rico_cmd", Command, self.callback)

    def callback(self, data):
        # data is of type tiago_msgs.msg.Command, i.e. it encapsulates an intent
        self.add(data)

    def execute(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))

        while True:
            if self.__shutdown__:
                return 'shutdown'

            if self.preempt_requested():
                self.service_preempt()
                print 'HearState: preemption'
                return 'preemption'

            self.__intent_list_lock__.acquire()
            unknown_items = self.conversation_interface.processIntents( self.__intent_list__ )

            if unknown_items:
                print 'unknown_items: ', unknown_items
                self.conversation_interface.addSpeakSentence(u'Nie rozumiem pytania')

            # TODO: react to unknown items
            self.__intent_list__ = set()
            self.__intent_list_lock__.release()

            if self.conversation_interface.hasSpeakSentence():
                return 'should_speak'

            if self.conversation_interface.isShutdown():
                print 'HearState preemption'
                return 'preemption'

            try:
                rospy.sleep(0.1)
            except rospy.ROSInterruptException:
                return 'error'

        raise Exception('Unreachable code')

    def add(self, item):
        self.__intent_list_lock__.acquire()
        self.__intent_list__.add( item )
        self.__intent_list_lock__.release()

class SpeakState(smach_rcprg.State):
    def __init__(self, conversation_interface):
        smach_rcprg.State.__init__(self, output_keys=[],
                                    outcomes=['ok', 'preemption', 'error', 'shutdown'])
        self.conversation_interface = conversation_interface
        self.rico_says_pub = rospy.Publisher('rico_says', String, queue_size=10)

    def execute(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))

        while True:
            if self.__shutdown__:
                return 'shutdown'

            if self.preempt_requested():
                self.service_preempt()
                print 'SpeakState: preemption'
                return 'preemption'

            sentence = self.conversation_interface.getSpeakSentence()

            if sentence is None:
                break

            # TODO: use ROS action to speak the sentences (with waiting for finish)
            self.rico_says_pub.publish( sentence )
            print 'Rico says: "' + sentence + '"'
            rospy.sleep(1.0)

            if self.conversation_interface.isShutdown() and not self.conversation_interface.hasSpeakSentence():
                print 'SpeakState preemption'
                return 'preemption'

            try:
                rospy.sleep(0.1)
            except rospy.ROSInterruptException:
                return 'error'

        return 'ok'

#    def add(self, item):
#        self.__items_lock__.acquire()
#        self.__items__.add( item )
#        self.__items_lock__.release()

class ConversationSM(smach_rcprg.StateMachine):
    def __init__(self, conversation_interface):
        smach_rcprg.StateMachine.__init__(self,
                                        outcomes=['PREEMPTED',
                                                    'FAILED',
                                                    'FINISHED', 'shutdown'])

        with self:
            smach_rcprg.StateMachine.add('Hear', HearState(conversation_interface),
                                    transitions={
                                        'should_speak':'Speak',
                                        'preemption':'PREEMPTED',
                                        'error':'FAILED',
                                        'shutdown':'shutdown'},
                                    remapping={ })

            smach_rcprg.StateMachine.add('Speak', SpeakState(conversation_interface),
                                    transitions={
                                        'ok':'Hear',
                                        'preemption':'PREEMPTED',
                                        'error':'FAILED',
                                        'shutdown':'shutdown'},
                                    remapping={ })

'''
    def updateAction(self, action_name, sm_goal):
        print 'ConversationSM.updateAction ' + action_name
        self.get_children()['Hear'].add( action_name )

    def newTaskCallback(self, task):
        print 'ConversationSM.newTaskCallback( "' + task.getName() + '")'

        if not task.getName() in ['q_load', 'q_current_task', 'ack', 'ack_i_gave', 'ack_i_took']:
            # Do nothing
            return

        self.get_children()['Hear'].add( task.getName() )
'''

class ConversationInterface:
    def __init__(self):
        self.__expected_items__ = set()
        self.__items__ = set()
        self.__autoremove_dict__ = {}
        self.__mutex__ = threading.Lock()
        self.__speak_list__ = []

        self.__shutdown__ = False
        self.__item_types__ = []

    def addExpected( self, expected, autoremove ):
        self.__mutex__.acquire()
        try:
            self.__expected_items__.add( expected )
            self.__autoremove_dict__[expected] = autoremove
        finally:
            self.__mutex__.release()

    def removeExpected( self, expected ):
        self.__mutex__.acquire()
        try:
            if expected in self.__expected_items__:
                self.__expected_items__.remove( expected )
            if expected in self.__items__:
                self.__items__.remove( expected )
        finally:
            self.__mutex__.release()

    def processIntents(self, intent_list):
        self.__mutex__.acquire()
        unknown_items = set()
        for intent in intent_list:
            name = self.getNameForIntent(intent.intent_name)
            if name in self.__expected_items__:
                self.__items__.add(name)
                if self.__autoremove_dict__[name]:
                    self.__expected_items__.remove(name)
            else:
                unknown_items.add(name)
        self.__mutex__.release()
        return unknown_items

    def consumeItem(self, item):
        self.__mutex__.acquire()
        if item in self.__items__:
            self.__items__.remove(item)
            consumed = True
        else:
            consumed = False
        self.__mutex__.release()
        return consumed

    def addSpeakSentence(self, sentence):
        assert isinstance(sentence, unicode)
        str_sentence = str(sentence.encode('utf-8'))
        print 'addSpeakSentence', type(sentence), sentence, type(str_sentence), str_sentence
        self.__mutex__.acquire()
        self.__speak_list__.append( str_sentence )
        self.__mutex__.release()

    def hasSpeakSentence(self):
        self.__mutex__.acquire()
        has = bool(self.__speak_list__)
        self.__mutex__.release()
        return has

    def getSpeakSentence(self):
        self.__mutex__.acquire()
        if self.__speak_list__:
            sentence = self.__speak_list__.pop(0)
        else:
            sentence = None
        self.__mutex__.release()
        return sentence

    def hasSpeakSentence(self):
        self.__mutex__.acquire()
        if self.__speak_list__:
            has_sentence = True
        else:
            has_sentence = False
        self.__mutex__.release()
        return has_sentence

    def setShutdown(self):
        self.__shutdown__ = True

    def isShutdown(self):
        return self.__shutdown__

    def addItemType(self, name, intent_name):
        self.__item_types__.append( (name, intent_name) )

    def getNameForIntent(self, intent_name):
        for name, in_name in self.__item_types__:
            if in_name == intent_name:
                return name
        return None

    def getIntentForName(self, intent_name):
        for nn, in_name in self.__item_types__:
            if nn == name:
                return in_name
        return None
