#!/usr/bin/env python
# encoding: utf8

import sys
import time
import threading

import rospy
import smach
import smach_ros
from std_msgs.msg import String
from TaskER.msg import Status, CMD
from TaskER.srv import CostConditions, SuspendConditions
from tiago_smach.ros_node_utils import get_node_names
import tiago_msgs.msg

class SmachShutdownManager:
    def __init__(self, main_sm, list_asw, list_sis):
        self.__main_sm__ = main_sm
        self.__list_asw__ = list_asw
        self.__list_sis__ = list_sis

    def on_shutdown(self):
        # Stop all introspection servers
        print 'SmachShutdownManager.on_shutdown: stopping all introspection servers'
        for sis in reversed(self.__list_sis__):
            sis.stop()

        # Stop all smach_ros action servers
        print 'SmachShutdownManager.on_shutdown: stopping all smach_ros action servers'
        for asw in reversed(self.__list_asw__):
            # This is a veru ugly hack:
            asw._action_server.action_server.started = False

            asw.wrapped_container.request_preempt()

        # Stop the main SM
        print 'SmachShutdownManager.on_shutdown: stopping the main SM'
        self.__main_sm__.request_preempt()

        #rospy.sleep(2.0)
class SuspendRequest:
    def __init__(self):
        self.req_data = String()
    def setData(self, data):
        self.req_data = data
    def getData(self):
        return self.req_data

class DynAgent:
    def __init__(self, da_name, da_id, da_type, ptf_csp):

        global debug
        debug = True
        self.name = da_name
        rospy.init_node(self.name)
        rospy.sleep(0.1)
        self.sub_task_state_cmd = rospy.Subscriber('/' + self.name + '/task_state_cmd', String, self.callbackTaskStateCmd)
        self.finished = False

        self.pub_diag = rospy.Publisher('/current_dyn_agent/diag', tiago_msgs.msg.DynAgentDiag, queue_size=10)
        # assign ptf update state
        self.ptf_csp = ptf_csp
        # set DA_ID used by TaskER
        self.da_id = da_id
        self.taskType = da_type
        self.exec_fsm_state = ""
        self.da_state = "init"
        self.terminateFlag = False
        self.startFlag = False
        self.ptf_csp(["scheduleParams", None])
        # object to send suspend request to the current task stage
        self.da_suspend_request = SuspendRequest()
        self.da_suspend_request.setData("suspension requirements from the task harmoniser")

    
    def startTask(self,data):
        self.startFlag = True

    def suspTask(self, data="suspension requirements from the task harmoniser"):
        print("\n","HOLD: ",str( data)+"\n")
        # propagate suspend request to exec FSM
        self.da_suspend_request.setData(data)
        self.main_sm.request_preempt()

    def updateStatus(self):
        global debug
        my_status = Status()
        my_status.da_id = self.da_id
        my_status.da_name = self.name
        my_status.da_state = self.da_state
        my_status.type = self.taskType
        my_status.schedule_params = self.ptf_csp(["scheduleParams", None])
        if debug:
            print("UPDATEING STATUS params of: "+str(self.name)+"\n"+str(my_status.schedule_params)+"\n")
        self.pub_status.publish(my_status) 
        # print("STATUS was sent"+"\n")
    def cmd_handler(self, data):
        global debug
        if data.recipient_name == self.name:
            if data.cmd == "start":
                # task will be started, so the interface to request start conditions by 'par' buffer is not required anymore
                self.cost_cond_srv.shutdown()
                self.startTask(data.data)
            elif data.cmd == "susp":
                self.susp_cond_srv.shutdown()
                self.cost_cond_srv = rospy.Service(cost_cond_name, CostConditions, self.startConditionHandler)
                self.suspTask(data.data)
            elif data.cmd == "resume":
                self.susp_cond_srv = rospy.Service(susp_cond_name, SuspendConditions, self.suspendConditionHandler)
                self.da_suspend_request.setData("resume")
            elif data.cmd == "terminate":
                self.main_sm.shutdownRequest()

    def suspendConditionHandler(self, req):
        return self.ptf_csp(["suspendCondition",req])
    def startConditionHandler(self, req):
        return self.ptf_csp(["startCondition",req])

    def run(self, main_sm):
        self.main_sm = main_sm

        #sis_main = smach_ros.IntrospectionServer('behaviour_server', self.main_sm, '/SM_BEHAVIOUR_SERVER')
        #sis_main.start()

        #ssm = SmachShutdownManager(self.main_sm, [], [sis_main])
        ssm = SmachShutdownManager(self.main_sm, [], [])
        # setup status interface for the task harmoniser
        self.pub_status = rospy.Publisher('TH/statuses', Status, queue_size=10)
        # subsribe to commands from the task harmoniser 

        node_namespace = rospy.get_name() + "/TaskER"
        start_name = node_namespace + "/startTask"
        self.cmd_subscriber = rospy.Subscriber(node_namespace+"/cmd", CMD, self.cmd_handler)
        cost_cond_name = node_namespace + "/get_cost_on_conditions"
        self.cost_cond_srv = rospy.Service(cost_cond_name, CostConditions, self.startConditionHandler)
        # Set shutdown hook
        rospy.on_shutdown( ssm.on_shutdown )

        thread_conn = threading.Thread(target=self.connectionCheckThread, args=(1,))
        thread_conn.start()
        # wait for start signal
        r = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.updateStatus()
            if self.startFlag:
                break 
            if self.terminateFlag:
                ssm.on_shutdown()
                return
            r.sleep()
        # setup suspend condition handler 
        susp_cond_name = node_namespace + "/get_suspend_conditions"
        self.susp_cond_srv = rospy.Service(susp_cond_name, SuspendConditions, self.suspendConditionHandler)
        # extend userdata with suspension request object
        self.main_sm.userdata.susp_data = self.da_suspend_request
        smach_thread = threading.Thread(target=self.main_sm.execute, args=(smach.UserData(),))
        smach_thread.start()
        print 'Smach thread is running'

        # Block until everything is preempted
        smach_thread.join()
        self.finished = True
        thread_conn.join()

        print 'Smach thread is finished'
        ssm.on_shutdown()

    def callbackTaskStateCmd(self, data):
        print 'DynAgent.callback'
        if data.data == 'abort':
            print 'DynAgent "' + self.name + '" received abort command'
            self.finished = True
            self.main_sm.shutdownRequest()

    def getActiveStates(self, sm):
        result = []
        if hasattr( sm, 'get_active_states' ):
            for state_name in sm.get_active_states():
                st = sm.get_children()[state_name]
                if hasattr( st, 'description' ) and not st.description is None:
                    description = st.description
                else:
                    description = ''
                result.append( (state_name, description) )
            for state_name in sm.get_active_states():
                st = sm.get_children()[state_name]
                result = result + self.getActiveStates( st )
        return result

    def connectionCheckThread(self, args):
        while not self.finished:
            active_ros_nodes = get_node_names()
            if not '/rico_task_harmonizer' in active_ros_nodes:
                print 'DynAgent "' + self.name + '" received has detected the task_harmonizer is dead'
                self.finished = True
                self.main_sm.shutdownRequest()
                break

            try:
                # Publish diagnostic information
                diag = tiago_msgs.msg.DynAgentDiag()
                diag.agent_name = self.name
                if self.startFlag:
                    rospy.sleep(2) 
                    active_states = self.getActiveStates( self.main_sm )
                    for state_name, state_desc in active_states:
                        diag.current_states.append( state_name )
                        diag.descriptions.append( state_desc )
                else:
                    diag.current_states.append("init")
                    diag.descriptions.append( "desc" )
                self.pub_diag.publish( diag )
            except Exception as e:
                print 'Detected exception in dynamic agent'
                print e
                self.finished = True
                self.main_sm.shutdownRequest()
                break

            time.sleep(0.2)
