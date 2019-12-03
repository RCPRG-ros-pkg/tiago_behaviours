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

NAVIGATION_MAX_TIME_S = 200

class SetNavParams(smach.State):
    def __init__(self):
        #print "navigation: SetNavParams(" + str(max_lin_vel) + ', ' + str(max_lin_accel) + ")"
        #self.max_lin_vel = max_lin_vel
        #self.max_lin_accel = max_lin_accel

        base_local_planner = rospy.get_param('/move_base/base_local_planner')
        self.local_planner_name = base_local_planner.split('/')[-1]

        self.dynparam_client = dynamic_reconfigure.client.Client('move_base/' + self.local_planner_name)

        smach.State.__init__(self, input_keys=['max_lin_vel_in', 'max_lin_accel_in'],
                             outcomes=['ok', 'preemption', 'error'])

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

    def execute(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))

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
    def __init__(self):
        self.current_pose = Pose()
        self.is_feedback_received = False
        self.move_base_status = GoalStatus.PENDING
        self.is_goal_achieved = False

        smach.State.__init__(self,
                             outcomes=['ok', 'preemption', 'error'],
                             input_keys=['nav_goal_pose'])

    def execute(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))

        # robot movement here - using Tiago move_base
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
                return 'error'

            rospy.sleep(0.1)

        # Here check move_base DONE status
        if self.move_base_status != GoalStatus.SUCCEEDED:
            return 'error'

        return 'ok'

    def move_base_feedback_cb(self, feedback):
        self.current_pose = feedback.base_position.pose
        self.is_feedback_received = True

    def move_base_done_cb(self, status, result):
        self.is_goal_achieved = True
        self.move_base_status = status

    def move_base_active_cb(self):
        return
