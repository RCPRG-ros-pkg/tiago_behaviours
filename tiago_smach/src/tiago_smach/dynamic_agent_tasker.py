#!/usr/bin/env python
# encoding: utf8

import sys
import time
import threading

import rospy
import smach
import smach_ros
from std_msgs.msg import String
from TaskER.msg import Status, CMD, ScheduleParams
from TaskER.srv import CostConditions, SuspendConditions
from tiago_smach.ros_node_utils import get_node_names
import tiago_msgs.msg
from std_srvs.srv import Trigger, TriggerRequest

class SmachShutdownManager:
    def __init__(self, dyn_agent, main_sm, list_asw, list_sis):
        self.__dyn_agent__ = dyn_agent
        self.__main_sm__ = main_sm
        self.__list_asw__ = list_asw
        self.__list_sis__ = list_sis

    def on_shutdown(self):
        print "DA MANAGER"
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

        

        #rospy.sleep(2.0)
class SuspendRequest:
    def __init__(self):
        self.req_data = []
    def setData(self, data):
        self.req_data = data
    def getData(self):
        return self.req_data
    def clearData(self):
        self.req_data = []

class DynAgent:
    def __init__(self, da_name, da_id, da_type, ptf_csp, da_state_name):

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
        self.da_id = int(da_id)
        self.taskType = da_type
        self.exec_fsm_state = ""
        self.da_state = ["init"]
        self.terminateFlag = False
        self.startFlag = False
        # self.process_ptf_csp(["scheduleParams", None])
        # object to send suspend request to the current task stage
        self.da_suspend_request = SuspendRequest()
        self.da_suspend_request.setData(["cmd","","param_name", "suspension requirements from the task harmoniser"])
        self.is_initialised = False

    def startTask(self,data):
        self.startFlag = True
        self.da_suspend_request.setData(data)

    def suspTask(self, data="suspension requirements from the task harmoniser"):
        print("\n","HOLD: ",str( data)+"\n")
        # propagate suspend request to exec FSM
        self.da_suspend_request.setData(data)
        # self.main_sm.request_preempt()
    def terminateDA(self):
        if not self.terminateFlag == True:
            if self.main_sm.is_running():
                if self.da_state[0] == "init" or self.getActiveStates( self.main_sm )[0][0] in ["Wait", "UpdateTask"]:
                    self.cmd_handler(CMD(recipient_name=self.name,cmd="terminate"))
                    my_status = Status()
                    my_status.da_id = self.da_id
                    my_status.da_name = self.name
                    my_status.type = self.taskType
                    self.da_state = ["END"]
                    my_status.da_state = self.da_state
                    self.pub_status.publish(my_status) 
                    self.terminateFlag = True
                else:
                    print "DA triggered self termination flag. It is in CMD state, so the flag triggers preemption"
                    self.suspTask(["cmd", "terminate"])
            else:
                print "DA -> TaskER Termination not required, because TaskER FSM is not running. Sending <END> state to THA"
                self.cmd_handler(CMD(recipient_name=self.name,cmd="terminate"))
                my_status = Status()
                my_status.da_id = self.da_id
                my_status.da_name = self.name
                my_status.type = self.taskType
                self.da_state = ["END"]
                my_status.da_state = self.da_state
                self.pub_status.publish(my_status) 
                self.terminateFlag = True
        else:
            print "termination flag was already handled"
    def process_ptf_csp(self, req):
        (flag, result) = self.ptf_csp(req)
        if flag == 'self-terminate':
            print "DA_tasker: SELF terminate"
            self.terminateDA()
            if req[0] == 'suspendCondition':
                return SuspendConditionsResponse()
            elif req[0] == 'CostConditions':
                return SuspendConditionsResponse()
            elif req[0] == 'scheduleParams':
                return ScheduleParams()
            else:
                return None
        else:
            return result
    def updateStatus(self):
        global debug
        if self.is_initialised == False :
            self.da_state = ['init']
        else:
            print "update, states: ", self.getActiveStates( self.main_sm )
            self.da_state = self.getActiveStates( self.main_sm )[0]

        my_status = Status()
        my_status.da_id = self.da_id
        my_status.da_name = self.name
        my_status.type = self.taskType
        result = self.process_ptf_csp(["scheduleParams", None])
        if result != 'self-terminate':
            my_status.schedule_params = result
        else:
            return
        my_status.da_state = self.da_state
        if debug:
            print("UPDATEING STATUS params of: "+str(self.name)+"\n of "+ str( self.taskType)+" type \n"+str(my_status.schedule_params)+"\n")
        self.pub_status.publish(my_status) 
        # print("STATUS was sent"+"\n")
    def cmd_handler(self, data):
        global debug
        print "DA got CMD: ", data
        if data.recipient_name == self.name:
            if data.cmd == "start":
                # task will be started, so the interface to request start conditions by 'par' buffer is not required anymore
                self.cost_cond_srv.shutdown()
                fsm_data = ["cmd", "start"]
                fsm_data.extend(data.data)
                self.startTask(fsm_data)
                self.hold_service = rospy.Service(self.node_namespace+"/hold_now", Trigger, lambda : None )
            elif data.cmd == "susp" and self.da_state[0] == "ExecFSM":
                self.hold_service.shutdown()
                self.start_service = rospy.Service(self.node_namespace+"/startTask", Trigger, lambda : None )
                self.susp_cond_srv.shutdown()
                self.cost_cond_srv = rospy.Service(self.cost_cond_name, CostConditions, self.startConditionHandler)
                fsm_data = ["cmd", "susp"]
                fsm_data.extend(data.data)
                self.suspTask(fsm_data)
            elif data.cmd == "resume" and self.da_state[0] == "Wait":
                self.start_service.shutdown()
                self.cost_cond_srv.shutdown()
                self.hold_service = rospy.Service(self.node_namespace+"/hold_now", Trigger, lambda : None )
                self.susp_cond_srv = rospy.Service(self.susp_cond_name, SuspendConditions, self.suspendConditionHandler)
                fsm_data = ["cmd", "resume"]
                fsm_data.extend(data.data)
                self.da_suspend_request.setData(fsm_data)
            elif data.cmd == "terminate":
                fsm_data = ["cmd", "terminate"]
                fsm_data.extend(data.data)
                self.da_suspend_request.setData(fsm_data)
                self.main_sm.request_preempt()
                self.main_sm.shutdownRequest()

    def suspendConditionHandler(self, req):
        return self.process_ptf_csp(["suspendCondition",req])
    def startConditionHandler(self, req):
        return self.process_ptf_csp(["startCondition",req])

    def run(self, main_sm, sis=None):
        self.main_sm = main_sm
        print "RUNNING DA"

        #sis_main = smach_ros.IntrospectionServer('behaviour_server', self.main_sm, '/SM_BEHAVIOUR_SERVER')
        #sis_main.start()

        #ssm = SmachShutdownManager(self.main_sm, [], [sis_main])

        # sis = smach_ros.IntrospectionServer(str("/"+self.name+"smach_view_server"), self.main_sm, self.name)
        # sis.start()
        ssm = SmachShutdownManager(self, self.main_sm, [], [])
        # setup status interface for the task harmoniser
        self.pub_status = rospy.Publisher('TH/statuses', Status, queue_size=10)
        # subsribe to commands from the task harmoniser 

        print "Setting DA"

        self.node_namespace = self.name + "/TaskER"
        start_name = self.node_namespace + "/startTask"
        self.cmd_subscriber = rospy.Subscriber("/TH/cmd", CMD, self.cmd_handler)
        self.cost_cond_name = self.node_namespace + "/get_cost_on_conditions"
        self.cost_cond_srv = rospy.Service(self.cost_cond_name, CostConditions, self.startConditionHandler)
        # Set shutdown hook
        rospy.on_shutdown( ssm.on_shutdown )
        if not self.terminateFlag:
            print "starting dignostic topic DA"
            thread_conn = threading.Thread(target=self.connectionCheckThread, args=(1,))
            thread_conn.start()
            print "started dignostic topic DA"
            # setup introspection server for smach viewer
            # extend userdata with suspension request object
            self.main_sm.userdata.susp_data = self.da_suspend_request
            smach_thread = threading.Thread(target=self.main_sm.execute, args=(smach.UserData(),))
            smach_thread.start()
            # wait for start signal
            r = rospy.Rate(1)                
            self.start_service = rospy.Service(self.node_namespace+"/startTask", Trigger, lambda : None )
            while not rospy.is_shutdown():
                print "RUNNING"
                self.updateStatus()
                if self.startFlag:
                    break 
                if self.terminateFlag:
                    print "RUNNING: TERM FLAG"
                    self.start_service.shutdown()
                    ssm.on_shutdown()
                    smach_thread.join()
                    thread_conn.join()
                    return
                r.sleep()
            self.start_service.shutdown()
            print 'Smach thread is running'
            self.is_initialised = True
            # setup suspend condition handler 
            self.susp_cond_name = self.node_namespace + "/get_suspend_conditions"
            self.susp_cond_srv = rospy.Service(self.susp_cond_name, SuspendConditions, self.suspendConditionHandler)
            # Block until everything is preempted
            smach_thread.join()
            print "SMACH JOINED"
            self.terminateDA()
            thread_conn.join()
            print "CONN Joined"
            self.terminateFlag = True
            print 'Smach thread is finished'
        else:
            print "DA -> have terminateFlag before TaskER FSM start"
        ssm.on_shutdown()
        print "DYN AGENT ENDED"

    def callbackTaskStateCmd(self, data):
        print 'DynAgent.callback'
        if data.data == 'abort':
            print 'DynAgent "' + self.name + '" received abort command'
            self.terminateFlag = True
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
        while not self.terminateFlag:
            active_ros_nodes = get_node_names()
            if not '/rico_task_harmonizer' in active_ros_nodes:
                print 'DynAgent "' + self.name + '" detected the task_harmonizer is dead'
                self.terminateDA()
                fsm_data = ["cmd", "terminate"]
                self.suspTask(fsm_data)
                self.main_sm.request_preempt()
                self.main_sm.shutdownRequest()
                break

            try:
                # Publish diagnostic information 
                diag = tiago_msgs.msg.DynAgentDiag()
                diag.agent_name = self.name
                if self.startFlag:
                    rospy.sleep(2) 
                    self.updateStatus() 
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
                self.terminateDA()
                self.main_sm.request_preempt()
                self.main_sm.shutdownRequest()
                break

            time.sleep(0.2)
