#!/usr/bin/env python

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
from geometry_msgs.msg import Pose
import std_srvs.srv as std_srvs
from std_msgs.msg import String

NAVIGATION_MAX_TIME_S = 100

class SayImGoingTo(smach.State):
    def __init__(self, is_simulated):
        smach.State.__init__(self, input_keys=['nav_goal_pose'],
                             outcomes=['ok', 'preemption', 'error'])

        self.rico_says_pub = rospy.Publisher('rico_says', String, queue_size=10)

    def execute(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))

        #self.rico_says_pub.publish( 'Jade do ["' + str(userdata.goods_name.goods_name) + '", dopelniacz]' );
        try:
            pose = userdata.nav_goal_pose.pose
        except:
            pose = userdata.nav_goal_pose

            self.rico_says_pub.publish( 'Jade do pozycji ' + str(pose.position.x) + ', ' + str(pose.position.y) );
        return 'ok'


class SetNavParams(smach.State):
    def __init__(self, is_simulated):
        self.is_simulated = is_simulated
        if is_simulated == False:
            base_local_planner = rospy.get_param('/move_base/base_local_planner')
            self.local_planner_name = base_local_planner.split('/')[-1]

            print 'SetNavParams: detected local planner: ' + self.local_planner_name

            self.dynparam_client = dynamic_reconfigure.client.Client('/move_base/' + self.local_planner_name)

            # Ensure that we know what we are doing.
            # Check if the given parameters are present in the configuration.
            config = self.dynparam_client.get_configuration()
            if self.local_planner_name == 'PalLocalPlanner':
                assert 'max_vel_x' in config
                assert 'acc_lim_x' in config
            elif self.local_planner_name == 'EBandPlannerROS':
                assert 'max_vel_lin' in config
                assert 'max_acceleration' in config
            elif self.local_planner_name == 'TebLocalPlannerROS':
                assert 'max_vel_x' in config
                assert 'acc_lim_x' in config
            else:
                raise Exception('Local planner "' + self.local_planner_name + '" is not supported.')

        smach.State.__init__(self, input_keys=['max_lin_vel_in', 'max_lin_accel_in'],
                             outcomes=['ok', 'preemption', 'error'])

    def execute(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))

        if self.is_simulated:
            rospy.sleep(1.0)
        else:
            max_lin_vel = userdata.max_lin_vel_in
            max_lin_accel = userdata.max_lin_accel_in

            # setting planner params so it the robot moves slowly
            # params = userdata.set_nav_params_params
            # Differrent planners have different parameters
            if self.local_planner_name == 'PalLocalPlanner':
                params = {
                    'max_vel_x': max_lin_vel,
                    'acc_lim_x': max_lin_accel
                }
            elif self.local_planner_name == 'EBandPlannerROS':
                params = {
                    'max_vel_lin': max_lin_vel,
                    'max_acceleration': max_lin_accel
                }
            elif self.local_planner_name == 'TebLocalPlannerROS':
                params = {
                    'max_vel_x': max_lin_vel,
                    'acc_lim_x': max_lin_accel
                }
            else:
                raise Exception('Local planner "' + self.local_planner_name + '" is not supported.')

            config = self.dynparam_client.update_configuration(params)

        return 'ok'

class MoveTo(smach.State):
    def __init__(self, is_simulated):
        self.current_pose = Pose()
        self.is_feedback_received = False
        self.move_base_status = GoalStatus.PENDING
        self.is_goal_achieved = False
        self.is_simulated = is_simulated

        smach.State.__init__(self,
                             outcomes=['ok', 'preemption', 'error', 'stall'],
                             input_keys=['nav_goal_pose'])

    def execute(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))

        if self.is_simulated:
            for i in range(50):
                if self.preempt_requested():
                    self.service_preempt()
                    return 'preemption'
                rospy.sleep(0.1)
            return 'ok'
        else:
            # robot movement here - using Tiago move_base
            try:
                pose = userdata.nav_goal_pose.pose
            except:
                pose = userdata.nav_goal_pose

            goal = MoveBaseGoal()
            goal.target_pose.pose = pose
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()

            client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            client.wait_for_server()

            # start moving
            client.send_goal(goal, self.move_base_done_cb, self.move_base_active_cb, self.move_base_feedback_cb)

            # action_feedback = GoActionFeedback()
            # action_result = GoActionResult()
            # action_result.result.is_goal_accomplished = False
            # userdata.nav_result = action_result.result

            start_time = rospy.Time.now()

            self.is_goal_achieved = False
            while self.is_goal_achieved == False:
                # action_feedback.feedback.current_pose = self.current_pose

                # userdata.nav_feedback = action_feedback.feedback
                # userdata.nav_actual_pose = self.current_pose

                end_time = rospy.Time.now()
                loop_time = end_time - start_time
                loop_time_s = loop_time.secs

                if loop_time_s > NAVIGATION_MAX_TIME_S:
                    # break the loop, end with error state
                    rospy.logwarn('State: Navigation took too much time, returning error')
                    client.cancel_all_goals()
                    return 'stall'

                if self.preempt_requested():
                    client.cancel_all_goals()
                    self.service_preempt()
                    return 'preemption'

                rospy.sleep(0.1)

            # Manage state of the move_base action server

            # Here check move_base DONE status
            if self.move_base_status == GoalStatus.PENDING:
                # The goal has yet to be processed by the action server
                raise Exception('Wrong move_base action status: "PENDING"')
            elif self.move_base_status == GoalStatus.ACTIVE:
                # The goal is currently being processed by the action server
                raise Exception('Wrong move_base action status: "ACTIVE"')
            elif self.move_base_status == GoalStatus.PREEMPTED:
                # The goal received a cancel request after it started executing
                #   and has since completed its execution (Terminal State)
                return 'preemption'
            elif self.move_base_status == GoalStatus.SUCCEEDED:
                # The goal was achieved successfully by the action server (Terminal State)
                return 'ok'
            elif self.move_base_status == GoalStatus.ABORTED:
                # The goal was aborted during execution by the action server due
                #    to some failure (Terminal State)
                return 'error'
            elif self.move_base_status == GoalStatus.REJECTED:
                # The goal was rejected by the action server without being processed,
                #    because the goal was unattainable or invalid (Terminal State)
                return 'error'
            elif self.move_base_status == GoalStatus.PREEMPTING:
                # The goal received a cancel request after it started executing
                #    and has not yet completed execution
                raise Exception('Wrong move_base action status: "PREEMPTING"')
            elif self.move_base_status == GoalStatus.RECALLING:
                # The goal received a cancel request before it started executing,
                #    but the action server has not yet confirmed that the goal is canceled
                raise Exception('Wrong move_base action status: "RECALLING"')
            elif self.move_base_status == GoalStatus.RECALLED:
                # The goal received a cancel request before it started executing
                #    and was successfully cancelled (Terminal State)
                return 'preemption'
            elif self.move_base_status == GoalStatus.LOST:
                # An action client can determine that a goal is LOST. This should not be
                #    sent over the wire by an action server
                raise Exception('Wrong move_base action status: "LOST"')
            else:
                raise Exception('Wrong move_base action status value: "' + str(self.move_base_status) + '"')

    def move_base_feedback_cb(self, feedback):
        self.current_pose = feedback.base_position.pose
        self.is_feedback_received = True

    def move_base_done_cb(self, status, result):
        self.is_goal_achieved = True
        self.move_base_status = status

    def move_base_active_cb(self):
        # Do nothing
        return

class ClearCostMaps(smach.State):
    def __init__(self, is_simulated):
        if is_simulated:
            self.clear_costmaps = None
        else:
            rospy.wait_for_service('/move_base/clear_costmaps')
            #try:
            self.clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', std_srvs.Empty)
            #except rospy.ServiceException, e:
            #    print "Service call failed: %s"%e
            #    self.clear_costmaps = None

        smach.State.__init__(self,
                             outcomes=['ok', 'preemption', 'error'])

    def execute(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))

        if not self.clear_costmaps is None:
            self.clear_costmaps()
        rospy.sleep(1.0)
        return 'ok'

class MoveToComplex(smach.StateMachine):
    def __init__(self, is_simulated):
        smach.StateMachine.__init__(self, outcomes=['FINISHED', 'PREEMPTED', 'FAILED'],
                                            input_keys=['nav_goal_pose'])

        with self:
            smach.StateMachine.add('SayImGoingTo', SayImGoingTo(is_simulated),
                                    transitions={'ok':'MoveTo', 'preemption':'PREEMPTED', 'error': 'FAILED'},
                                    remapping={'nav_goal_pose':'nav_goal_pose'})

            smach.StateMachine.add('MoveTo', MoveTo(is_simulated),
                                    transitions={'ok':'FINISHED', 'preemption':'PREEMPTED', 'error': 'FAILED', 'stall':'ClearCostMaps'},
                                    remapping={'nav_goal_pose':'nav_goal_pose'})

            smach.StateMachine.add('ClearCostMaps', ClearCostMaps(is_simulated),
                                    transitions={'ok':'MoveTo', 'preemption':'PREEMPTED', 'error': 'FAILED'})
