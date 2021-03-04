import copy
import threading
import rospy
import actionlib

import control_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg

class TiagoHeadController:
    def __init__(self,
                 pointing_frame='xtion_optical_frame',
                 pointing_axis=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=1.0)):

        self.pointing_axis = pointing_axis
        self.pointing_frame = pointing_frame

        # initialize publisher for setting services
        self.client = actionlib.SimpleActionClient('head_controller/point_head_action', control_msgs.msg.PointHeadAction)

        # Wait for server for 10 seconds.
        # Crash, if there is no such action.
        assert self.client.wait_for_server(timeout=rospy.Duration(5))

    def __send_goal(self, pointting_frame, pointing_axis, target_point, min_duration):
        point = geometry_msgs.msg.PointStamped()
        point.header.frame_id = 'base_link'
        point.header.stamp = rospy.Time.now()
        point.point = target_point

        goal = control_msgs.msg.PointHeadGoal()
        goal.pointing_frame = pointting_frame
        goal.pointing_axis = pointing_axis
        goal.min_duration.secs = min_duration
        goal.target = point

        # Callbacks are not handled right now
        self.client.send_goal(goal)

    def tilt_down(self, min_time=2.0):
        target_point = geometry_msgs.msg.Point(1.0, 0.0, 0.1)
        self.__send_goal(self.pointing_frame, self.pointing_axis, target_point, min_time)

    def tilt_up(self, min_time=2.0):
        target_point = geometry_msgs.msg.Point(1.0, 0.0, 1.75)
        self.__send_goal(self.pointing_frame, self.pointing_axis, target_point, min_time)

    def tilt_forward(self, min_time=2.0):
        target_point = geometry_msgs.msg.Point(1.0, 0.0, 1.25)
        self.__send_goal(self.pointing_frame, self.pointing_axis, target_point, min_time)

    def turn_to_point(self, point, min_time=2.0):
        self.__send_goal(self.pointing_frame, self.pointing_axis, point, min_time)

class TiagoTorsoController:
    def __init__(self):
        # initialize publisher for setting services
        self.client = actionlib.SimpleActionClient('torso_controller/follow_joint_trajectory', control_msgs.msg.FollowJointTrajectoryAction)

        # Wait for server for 10 seconds.
        # Crash, if there is no such action.
        assert self.client.wait_for_server(timeout=rospy.Duration(5))

        self.__lock__ = threading.Lock()
        self.current_height = None
        self.initialised = False
        self.sub = rospy.Subscriber("/torso_controller/state", control_msgs.msg.JointTrajectoryControllerState, self.callback)

    def callback(self, data):
        self.__lock__.acquire()
        self.current_height = copy.copy(data)
        self.initialised = True
        self.__lock__.release()

    def __send_goal(self, height):
        jtp = trajectory_msgs.msg.JointTrajectoryPoint()
        jtp.time_from_start = rospy.Duration(2)
        jtp.positions.append(height)

        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        goal.trajectory.joint_names.append('torso_lift_joint')
        goal.trajectory.header.stamp = rospy.Time.now()
        goal.trajectory.points.append(jtp)

        # Callbacks are not handled right now
        self.client.send_goal(goal)

    def set_torso_height(self, height):
        self.__send_goal(height)

    def get_torso_height(self):
        i=0
        self.__lock__.acquire()
        if self.current_height is None:
            while not self.initialised:
                self.__lock__.release()
                print "waiting for torso controller to initialise..."
                rospy.sleep(1)
                self.__lock__.acquire()
                if i > 6:
                    break
            if self.current_height is None:
                current_height = None
            else:
                current_height = copy.copy(self.current_height.actual.positions[0])
            self.__lock__.release()
        else:
            current_height = copy.copy(self.current_height.actual.positions[0])
            self.__lock__.release()
        return current_height
