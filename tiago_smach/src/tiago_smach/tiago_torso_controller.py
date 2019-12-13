import copy
import threading
import rospy
import actionlib

from control_msgs.msg import JointTrajectoryControllerState, FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from geometry_msgs.msg import Point, PointStamped, Vector3
from pal_interaction_msgs.msg import *
from trajectory_msgs.msg import *

class TiagoHeadController:
    def __init__(self,
                 pointing_frame='xtion_optical_frame',
                 pointing_axis=Vector3(x=0.0, y=0.0, z=1.0)):

        self.pointing_axis = pointing_axis
        self.pointing_frame = pointing_frame

        # initialize publisher for setting services
        self.client = actionlib.SimpleActionClient('head_controller/point_head_action', PointHeadAction)

        # Wait for server for 10 seconds.
        # Crash, if there is no such action.
        assert self.client.wait_for_server(timeout=rospy.Duration(5))

    def __send_goal(self, pointting_frame, pointing_axis, target_point, min_duration):
        point = PointStamped()
        point.header.frame_id = 'base_link'
        point.header.stamp = rospy.Time.now()
        point.point = target_point

        goal = PointHeadGoal()
        goal.pointing_frame = pointting_frame
        goal.pointing_axis = pointing_axis
        goal.min_duration.secs = min_duration
        goal.target = point

        # Callbacks are not handled right now
        self.client.send_goal(goal)

    def tilt_down(self, min_time=2.0):
        target_point = Point(1.0, 0.0, 0.1)
        self.__send_goal(self.pointing_frame, self.pointing_axis, target_point, min_time)

    def tilt_up(self, min_time=2.0):
        target_point = Point(1.0, 0.0, 1.75)
        self.__send_goal(self.pointing_frame, self.pointing_axis, target_point, min_time)

    def tilt_forward(self, min_time=2.0):
        target_point = Point(1.0, 0.0, 1.25)
        self.__send_goal(self.pointing_frame, self.pointing_axis, target_point, min_time)

    def turn_to_point(self, point, min_time=2.0):
        self.__send_goal(self.pointing_frame, self.pointing_axis, point, min_time)


class TiagoSpeechController:
    def __init__(self, lang='en_GB'):
        self.lang = lang

        # initialize publisher for setting services
        self.client = actionlib.SimpleActionClient('tts', TtsAction)

        # Wait for server for 10 seconds.
        # Crash, if there is no such action.
        #assert self.client.wait_for_server(timeout=rospy.Duration(10))

        if not self.client.wait_for_server(timeout=rospy.Duration(5)):
            self.client = None

    def __send_goal(self, lang, text_to_speech, delay):
        # TODO: use artificial action server for simulated system
        if self.client is None:
            print "TiagoSpeechController.__send_goal: not implemented for simulated system!"
            return

        goal = TtsGoal()
        goal.rawtext.lang_id = lang
        goal.rawtext.text = text_to_speech
        goal.wait_before_speaking = delay

        # Callbacks are not handled right now
        self.client.send_goal(goal)

    def tts(self, text_to_speech, delay=0.0):
        self.__send_goal(self.lang, text_to_speech, delay)


class TiagoTorsoController:
    def __init__(self):
        # initialize publisher for setting services
        self.client = actionlib.SimpleActionClient('torso_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

        # Wait for server for 10 seconds.
        # Crash, if there is no such action.
        assert self.client.wait_for_server(timeout=rospy.Duration(5))

        self.__lock__ = threading.Lock()
        self.current_height = None
        self.sub = rospy.Subscriber("/torso_controller/state", JointTrajectoryControllerState, self.callback)

    def callback(self, data):
        self.__lock__.acquire()
        self.current_height = copy.copy(data)
        self.__lock__.release()

    def __send_goal(self, height):
        jtp = JointTrajectoryPoint()
        jtp.time_from_start = rospy.Duration(2)
        jtp.positions.append(height)

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names.append('torso_lift_joint')
        goal.trajectory.header.stamp = rospy.Time.now()
        goal.trajectory.points.append(jtp)

        # Callbacks are not handled right now
        self.client.send_goal(goal)

    def set_torso_height(self, height):
        self.__send_goal(height)

    def get_torso_height(self):
        self.__lock__.acquire()
        if self.current_height is None:
            current_height = None
        else:
            current_height = copy.copy(self.current_height.actual.positions[0])
        self.__lock__.release()
        return current_height
