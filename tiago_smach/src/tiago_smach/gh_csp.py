#!/usr/bin/env python
# encoding: utf8

import math
import rospy

from move_base_msgs.msg import *
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose


import task_manager

from TaskER.msg import RobotResource, ScheduleParams
from nav_msgs.srv import *
from nav_msgs.msg import *


import tiago_kb.places_xml as kb_p
from tiago_smach.task_manager import PoseDescription

class GH_csp():
    def __init__(self, human_name):
        self.human_name = human_name
        if self.human_name == "John":
            # walk factor for human 0
            self.factor_walk = 0.15
            # sit factor for human 0
            self.factor_sit = 0.3
            # stand factor for human 0
            self.factor_stand = 0.5
        else:
            # walk factor for human 1
            self.factor_walk = 0.2
            # sit factor for human 1
            self.factor_sit = 0.3
            # stand factor for human 1
            self.factor_stand = 0.15
    def get_robot_pose(self, sim_mode, kb_places, robot_name):
        if sim_mode == 'sim':
            pose = makePose(0, 0, 0)
        else:
            if sim_mode == 'real':
                mc_name = 'real'
            elif sim_mode == 'gazebo':
                mc_name = 'sim'
            try:
                pl = kb_places.getPlaceByName(robot_name, mc_name)
            except:
                print 'UnderstandGoal place_name is not valid'
                return 'error'

            pt_dest = pl.getPt()
            norm = pl.getN()
            return makePose(pt_dest[0], pt_dest[1], math.atan2(norm[1], norm[0]))
    def get_dest_pose(self, sim_mode, kb_places, place_name, pt_start):

        if sim_mode == 'sim':
            pose = makePose(0, 0, 0)
        else:
            if sim_mode == 'real':
                mc_name = 'real'
            elif sim_mode == 'gazebo':
                mc_name = 'sim'

            if pl.getType() == 'point':
                pt_dest = pl.getPt()
                norm = pl.getN()
                print "NORM_0: ", norm[0]
                print "NORM_1: ", norm[1]
                angle_dest = -math.atan2(norm[1], norm[0])
                pt = pt_dest
                pt_dest = (pt_dest[0]+norm[0], pt_dest[1]+norm[1])
                print 'UnderstandGoal place type: point'
                print 'pt: {}, pt_dest: {}, norm: {}, angle_dest: {}'.format(pt, pt_dest, norm, angle_dest)
            elif pl.getType() == 'volumetric':
                pt_dest = self.kb_places.getClosestPointOfPlace(pt_start, pl.getId(), mc_name)
                angle_dest = 0.0
            else:
                raise Exception('Unknown place type: "' + pl.getType() + '"')
            pose = makePose(pt_dest[0], pt_dest[1], angle_dest)
        return pose

    def get_schedule_params(self, human_posture, guide_dest):
        sp = ScheduleParams()
        print "CALC COST"

        path_client = rospy.ServiceProxy('/move_base/GlobalPlanner/make_plan', GetPlan)
        req_path = GetPlanRequest()
        ros_time = rospy.Time()
        self.robot_trans = []
        human_trans = []
        robot_rot = []
        human_rot = []
        rate = rospy.Rate(5.0)
        while not rospy.is_shutdown():
            print "in while"
            try:
                listener.waitForTransform('/map', '/base_link', ros_time, rospy.Duration(8.0))
                (self.robot_trans,robot_rot) = listener.lookupTransform('/map', '/base_link', ros_time)
                
                listener.waitForTransform('/map', self.actor_name, ros_time, rospy.Duration(8.0))
                (human_trans,human_rot) = listener.lookupTransform('/map', self.actor_name, ros_time)
                if len(self.robot_trans) != 0 and len(human_trans) != 0:
                    break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rate.sleep()
                continue
        human_poses[human_id] = [human_trans[0],human_trans[1],0, human_rot[0], human_rot[1], human_rot[2]]
        human_last_poses[human_id] = human_poses[human_id]
        path_distance = 0
        req_path.start.header.frame_id = "map"
        req_path.start.pose.position.x = self.robot_trans[0]
        req_path.start.pose.position.y = self.robot_trans[1]
        req_path.start.pose.position.z = self.robot_trans[2]
        req_path.start.pose.orientation.x = robot_rot[0]
        req_path.start.pose.orientation.y = robot_rot[1]
        req_path.start.pose.orientation.z = robot_rot[2]
        req_path.start.pose.orientation.w = robot_rot[3]
        req_path.goal.header.frame_id = "map"
        req_path.goal.pose.position.x = human_trans[0]
        req_path.goal.pose.position.y = human_trans[1]
        req_path.goal.pose.position.z = human_trans[2]
        req_path.goal.pose.orientation.x = 0
        req_path.goal.pose.orientation.y = 0
        req_path.goal.pose.orientation.z = 0
        req_path.goal.pose.orientation.w = 0
        resp = path_client(req_path)
        # print "PATH LENGTH: ",len(resp.plan.poses)
        path_distance_approach = 0
        if len(resp.plan.poses) > 2:
            for i in range(len(resp.plan.poses)-1):
                # print "POSE: ",resp.plan.poses[i]
                path_distance_approach += math.sqrt(math.pow((resp.plan.poses[i+1].pose.position.x - resp.plan.poses[i].pose.position.x),2) + pow((resp.plan.poses[i+1].pose.position.y - resp.plan.poses[i].pose.position.y), 2))
        
        req_path.start.header.frame_id = "map"
        req_path.start.pose.position.x = human_trans[0]
        req_path.start.pose.position.y = human_trans[1]
        req_path.start.pose.position.z = human_trans[2]
        req_path.start.pose.orientation.x = human_rot[0]
        req_path.start.pose.orientation.y = human_rot[1]
        req_path.start.pose.orientation.z = human_rot[2]
        req_path.start.pose.orientation.w = human_rot[3]
        req_path.goal.header.frame_id = "map"
        req_path.goal.pose.position.x = human_dests[human_id][0]
        req_path.goal.pose.position.y = human_dests[human_id][1]
        req_path.goal.pose.position.z = human_dests[human_id][2]
        req_path.goal.pose.orientation.x = 0
        req_path.goal.pose.orientation.y = 0
        req_path.goal.pose.orientation.z = 0
        req_path.goal.pose.orientation.w = 0
        resp = path_client(req_path)
        # print "PATH LENGTH: ",len(resp.plan.poses)
        path_distance_guide = 0
        if len(resp.plan.poses) > 2:
            for i in range(len(resp.plan.poses)-1):
                # print "POSE: ",resp.plan.poses[i]
                path_distance_guide += math.sqrt(math.pow((resp.plan.poses[i+1].pose.position.x - resp.plan.poses[i].pose.position.x),2) + pow((resp.plan.poses[i+1].pose.position.y - resp.plan.poses[i].pose.position.y), 2))
        if human_posture == "walk":
            sp.cost_per_sec = factor_walk[human_id]
            cost = path_distance_guide * factor_walk[human_id]
        if human_posture == "stand":
            sp.cost_per_sec = factor_stand[human_id]
            cost = path_distance_approach*factor_stand[human_id] + path_distance_guide * factor_walk[human_id]
        if human_posture == "sit":
            sp.cost_per_sec = factor_sit[human_id]
            cost = path_distance_approach*factor_sit[human_id] + path_distance_guide * factor_walk[human_id]
        if human_posture == "ZERO":
            sp.cost_per_sec = 0
            cost = 0
        print "COST: ", cost
        sp.cost = cost
        sp.completion_time = path_distance_approach + path_distance_guide
        sp.final_resource_state.robot_position = req_path.goal.pose.position
        return sp
