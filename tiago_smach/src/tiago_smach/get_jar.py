#!/usr/bin/env python2

import rospy
import math
import tf2_ros
import PyKDL
import copy
from threading import Thread

from geometry_msgs.msg import Twist
from omnivelma_driver.srv import STPT
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive

from velma_common import *
from rcprg_planner import *
from rcprg_ros_utils import exitError, MarkerPublisher

from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
import tf_conversions.posemath as pm

class MarkerPublisherThread:
    def threaded_function(self, obj):
        pub = MarkerPublisher("attached_objects")
        while not self.stop_thread:
            pub.publishSinglePointMarker(PyKDL.Vector(), 1, r=1, g=0, b=0, a=1, namespace='default', frame_id=obj.link_name, m_type=Marker.CYLINDER, scale=Vector3(0.02, 0.02, 1.0), T=pm.fromMsg(obj.object.primitive_poses[0]))
            try:
                rospy.sleep(0.1)
            except:
                break

        try:
            pub.eraseMarkers(0, 10, namespace='default')
            rospy.sleep(0.5)
        except:
            pass

    def __init__(self, obj):
        self.thread = Thread(target = self.threaded_function, args = (obj, ))

    def start(self):
        self.stop_thread = False
        self.thread.start()

    def stop(self):
        self.stop_thread = True
        self.thread.join()

class SetpointGenerator():
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

    def generate(self, destinationFrame, safetyTranslation):
        rospy.sleep(0.5)
        msg = self.tfBuffer.lookup_transform("map", destinationFrame, rospy.Time(0))
        rot = msg.transform.rotation
        setpointRot = PyKDL.Rotation.Quaternion(rot.x, rot.y, rot.z, rot.w)
        trans = msg.transform.translation
        destinationFrame = PyKDL.Frame(setpointRot, PyKDL.Vector(trans.x, trans.y, trans.z))
        setpointVector = destinationFrame * safetyTranslation
        setpointRot.DoRotZ(math.pi)
        result = (setpointVector.x(), setpointVector.y(), setpointRot.GetRPY()[2])
        print(result)
        return result


class VelmaTaskExecutor():
    def __init__(self):
        self.velma = VelmaInterface()
        if not self.velma.waitForInit(timeout_s=10.0):
            raise Exception("Could not initialize VelmaInterface")

        if self.velma.enableMotors() != 0:
            raise Exception("Could not enable motors")

        self.velma.startHomingHP()
        if self.velma.waitForHP() != 0:
            raise Exception("Could not home head pan motor")

        self.velma.startHomingHT()
        if self.velma.waitForHT() != 0:
            raise Exception("Could not home head tilt motor")

        diag = self.velma.getCoreCsDiag()
        if not diag.motorsReady():
            raise Exception("Motors must be homed and ready to use for this test.")   

        self.omniplannerServiceName = "/omniplanner/go_to_stpt"
        self.omniplannerService = rospy.ServiceProxy(self.omniplannerServiceName, STPT)

        self.trajectoryPlanner = Planner(self.velma.maxJointTrajLen())
        if not self.trajectoryPlanner.waitForInit():
            raise Exception("Could not initialize trajectory planner")
        self.octomapListener = OctomapListener("/octomap_binary")
        rospy.sleep(1.0)

        octomap = self.octomapListener.getOctomap(timeout_s=5.0)
        self.trajectoryPlanner.processWorld(octomap)

        self.catchingJointState = {
            'torso_0_joint':0,
            'right_arm_0_joint': 0.0,   
            'left_arm_0_joint':0.3,
            'right_arm_1_joint':-1.9,   
            'left_arm_1_joint':1.8,
            'right_arm_2_joint':1.5,   
            'left_arm_2_joint':-1.25,
            'right_arm_3_joint':1.5,   
            'left_arm_3_joint':-0.85,
            'right_arm_4_joint':0,      
            'left_arm_4_joint':0,
            'right_arm_5_joint':-1.6,   
            'left_arm_5_joint':0.5,
            'right_arm_6_joint':0.0,      
            'left_arm_6_joint': 0.0
        }

        self.acquiringJointState = {
            'torso_0_joint': 0, 
            'right_arm_0_joint': 0.2176005457580103,   
            'left_arm_0_joint': 0.2345004080527655,
            'right_arm_1_joint': -1.9107791904878497,  
            'left_arm_1_joint': 1.8034769374904756,
            'right_arm_2_joint': 1.2409542924753767,   
            'left_arm_2_joint': -1.1982341925787994,
            'right_arm_3_joint': 1.4842204142092719,   
            'left_arm_3_joint': -0.8278483633253793, 
            'right_arm_4_joint': 0.2525831592128146,   
            'left_arm_4_joint': 0.07257063733648089,
            'right_arm_5_joint': -1.5390250000127208,  
            'left_arm_5_joint': 0.4699180006050142,
            'right_arm_6_joint': -0.21426825617036566, 
            'left_arm_6_joint': -0.0703725749418421,            
        }

        self.deliveringJointState =  {
            'torso_0_joint': 0.0,

            'left_arm_0_joint': 0.0,   
            'left_arm_1_joint':2.0,
            'left_arm_2_joint':-1.5, 
            'left_arm_3_joint':-1.65,   
            'left_arm_4_joint':0,
            'left_arm_5_joint':1.5,  
            'left_arm_6_joint': 0.0,

            'right_arm_0_joint':-0.3,   
            'right_arm_1_joint':-1.8,   
            'right_arm_2_joint':1.25,   
            'right_arm_3_joint':0.85,   
            'right_arm_4_joint':0,      
            'right_arm_5_joint':-0.5,   
            'right_arm_6_joint':0    
        }

        # self.grabbingJointState = {
        #   'left_arm_2_joint': -2.0355836822083657, 
        #   'right_arm_2_joint': 0.3709267067333251, 
        #   'right_arm_0_joint': 0.46569890384071055, 
        #   'right_arm_4_joint': 0.2241366773284909,  
        #   'right_arm_5_joint': -0.5316931184645763,
        #   'right_arm_3_joint': 0.38176870047166345, 
        #   'left_arm_0_joint': -0.15171555729988803, 
        #   'left_arm_6_joint': 0.4899765008800134,
        #   'right_arm_1_joint': -1.931444821966154, 
        #   'left_arm_1_joint': 1.9500022141952806, 
        #   'right_arm_6_joint': 0.188264943430854, 
        #   'torso_0_joint': -0.6293638853027678,
        #   'left_arm_3_joint': -1.0594496666438027, 
        #   'left_arm_5_joint': 1.7238910428595622, 
        #   'left_arm_4_joint': -0.06248503443106635
        # }

        # self.grabbingJointState = {
        #   'left_arm_2_joint': -1.3355836822083657, 
        #   'right_arm_2_joint': 0.3709267067333251, 
        #   'right_arm_0_joint': 0.46569890384071055, 
        #   'right_arm_4_joint': 0.2241366773284909,  
        #   'right_arm_5_joint': -0.5316931184645763,
        #   'right_arm_3_joint': 0.78176870047166345,
        #   'left_arm_0_joint': 0.20171555729988803, 
        #   'left_arm_6_joint': 0.5899765008800134,
        #   'right_arm_1_joint': -1.931444821966154, 
        #   'left_arm_1_joint': 1.9500022141952806, 
        #   'right_arm_6_joint': 0.188264943430854, 
        #   'torso_0_joint': -0.5293638853027678,
        #   'left_arm_3_joint': -1.6094496666438027, 
        #   'left_arm_5_joint': 1.7238910428595622, 
        #   'left_arm_4_joint': -0.06248503443106635
        # }

        self.grabbingJointState = {
            'torso_0_joint':-0.45,

            'left_arm_0_joint':-0.2,   
            'left_arm_1_joint':1.8,
            'left_arm_2_joint':-0.9, 
            'left_arm_3_joint':-1.65,   
            'left_arm_4_joint':0,
            'left_arm_5_joint':1.5,  
            'left_arm_6_joint': 0.8,

            'right_arm_0_joint':-0.3,   
            'right_arm_1_joint':-1.8,   
            'right_arm_2_joint':1.25,   
            'right_arm_3_joint':0.85,   
            'right_arm_4_joint':0,      
            'right_arm_5_joint':-0.5,   
            'right_arm_6_joint':0    
        }

        self.startingJointState = {
            'torso_0_joint':0,
            'right_arm_0_joint':-0.3,   
            'left_arm_0_joint':0.3,
            'right_arm_1_joint':-1.8,   
            'left_arm_1_joint':1.8,
            'right_arm_2_joint':1.25,   
            'left_arm_2_joint':-1.25,
            'right_arm_3_joint':0.85,   
            'left_arm_3_joint':-0.85,
            'right_arm_4_joint':0,      
            'left_arm_4_joint':0,
            'right_arm_5_joint':-0.5,   
            'left_arm_5_joint':0.5,
            'right_arm_6_joint':0,      
            'left_arm_6_joint': 0
        }


    def setJointImpedanceMode(self):
        self.velma.moveJointImpToCurrentPos(start_time=0.2)
        error = self.velma.waitForJoint()
        if error != 0:
            raise Exception("The action should have ended without error, but the error code is {}".format(error))

        rospy.sleep(0.5)
        diag = self.velma.getCoreCsDiag()
        if not diag.inStateJntImp():
            raise Exception("Core CS should be in joint impedance mode but it is not")

    def setCartesianImpedanceMode(self):
        if not self.velma.moveCartImpRightCurrentPos(start_time=0.2):
            raise Exception("Could not set CartImp mode for right lwr")

        if self.velma.waitForEffectorRight() != 0:
            raise Exception("Right effector error")

        if not self.velma.moveCartImpLeftCurrentPos(start_time=0.2):
            raise Exception("Could not set CartImp mode for left lwr")

        if self.velma.waitForEffectorLeft() != 0:
            raise Exception("Left effector error")

        rospy.sleep(0.5)
        diag = self.velma.getCoreCsDiag()
        if not diag.inStateCartImp():
            raise Exception("Core CS should be in cartesian impedance mode but it is not")

    def moveBodyToStartingPosition(self):
        startingJointState = {
            'torso_0_joint':0,
            'right_arm_0_joint':-0.3,   
            'left_arm_0_joint':0.3,
            'right_arm_1_joint':-1.8,   
            'left_arm_1_joint':1.8,
            'right_arm_2_joint':1.25,   
            'left_arm_2_joint':-1.25,
            'right_arm_3_joint':0.85,   
            'left_arm_3_joint':-0.85,
            'right_arm_4_joint':0,      
            'left_arm_4_joint':0,
            'right_arm_5_joint':-0.5,   
            'left_arm_5_joint':0.5,
            'right_arm_6_joint':0,      
            'left_arm_6_joint': 0
        }

        self.velma.moveJoint(startingJointState, 3.0, start_time=0.5, position_tol=10.0/180.0*math.pi)
        if self.velma.waitForJoint() != 0:
            raise Exception("Moving body failed")

    def moveHeadToStartingPosition(self):
        startingHeadState = (0, 0)
        self.velma.moveHead(startingHeadState, 3.0, start_time=0.5)
        if self.velma.waitForHead() != 0:
            raise Exception("Moving head failed")
        rospy.sleep(0.5)

        if not isHeadConfigurationClose(self.velma.getHeadCurrentConfiguration(), startingHeadState, 0.1):
            raise Exception("Head too far from destination")

    def moveToStartingPosition(self):
        self.setJointImpedanceMode()
        self.moveBodyToStartingPosition()
        self.moveHeadToStartingPosition()

    def hideHands(self):
        handDesiredState = [0.5*math.pi, 0.5*math.pi, 0.5*math.pi, math.pi]

        self.velma.moveHandRight(handDesiredState, [1, 1, 1, 1], [2000, 2000, 2000, 2000], 1000, hold=True)
        self.velma.moveHandLeft(handDesiredState, [1, 1, 1, 1], [2000, 2000, 2000, 2000], 1000, hold=True)
        if self.velma.waitForHandRight() != 0 and self.velma.waitForHandLeft() != 0:
            raise Exception("Could not hide hands")

    def driveToPointOnMap(self, setpoint):
        (x, y, yaw) = setpoint
        setpoint = STPT
        setpoint.x = x
        setpoint.y = y
        setpoint.theta = yaw
        setpoint.linear_velocity = 0.0
        setpoint.angular_velocity = 0.0

        response = self.omniplannerService(setpoint)
    
    def prepareForGrip(self, destinationFrame):
        T_Wo_Cabinet = self.velma.getTf("Wo", destinationFrame)
        T_B_Cabinet = self.velma.getTf("B", destinationFrame)

        cabinetX = T_Wo_Cabinet.p[0]
        cabinetY = T_Wo_Cabinet.p[1]
        cabinetZ = T_Wo_Cabinet.p[2]

        torsoAngle = math.atan2(cabinetY, cabinetX)
        if torsoAngle > math.pi /2:
            torsoAngle = math.pi /2 - 0.1
        elif torsoAngle < -math.pi /2:
            torsoAngle = -math.pi /2 + 0.1
        else:
            pass

        desiredJointState = copy.deepcopy(self.acquiringJointState)
        desiredJointState["torso_0_joint"] = torsoAngle

        self.velma.moveJoint(desiredJointState, 2.0, start_time=0.5, position_tol=10.0/180.0*math.pi)

        if self.velma.waitForJoint() != 0:
            raise Exception("Could not move joints")

        rospy.sleep(0.5)
        lastJointState = self.velma.getLastJointState()
        if not isConfigurationClose(desiredJointState, lastJointState[1]):
            raise Exception("Could not acquire desired joint state")

        self.setCartesianImpedanceMode()
        return T_B_Cabinet

    def resolveRelativePosition(self, tf, deltaX, deltaY, deltaZ):
        (rotX, rotY, rotZ) = tf.M.GetRPY()
        posX = tf.p.x() + math.cos(rotZ)*deltaX - math.sin(rotZ)*deltaY
        posY = tf.p.y() + math.sin(rotZ)*deltaX + math.cos(rotZ)*deltaY
        posZ = tf.p.z() + deltaZ
        yaw = rotZ - math.pi
        if yaw < -math.pi:
            yaw += 2*math.pi
        return [posX, posY, posZ, yaw]

    def moveRelativeToInCartImpMode(self, tf, x, y, z, angle, tol=10):
        (setpointX, setpointY, setpointZ, setpointYaw) = self.resolveRelativePosition(tf, x, y, z)
        targetFrame = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, setpointYaw+angle), PyKDL.Vector(setpointX, setpointY, setpointZ))
        result = self.moveInCartesianImpedanceMode(targetFrame, tol)
        return result

    def moveInCartesianImpedanceMode(self, T_B_dest, tol=10):
        if not self.velma.moveCartImpRight([T_B_dest], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.1, path_tol=PyKDL.Twist(PyKDL.Vector(tol, tol, tol), PyKDL.Vector(tol, tol, tol))):
            raise Exception("Could not move in cartesian impedance mode")
        if self.velma.waitForEffectorRight() != 0:
            if not self.velma.moveCartImpRightCurrentPos(start_time=0.01):
                raise Exception("Could not make it to given position")
            return False
        else:
            return True

    def setRightLWRImpedance(self, imp_p_x, imp_p_y, imp_p_z, imp_r_x, imp_r_y, imp_r_z):
        if not self.velma.moveCartImpRight(None, None, None, None, [PyKDL.Wrench(PyKDL.Vector(imp_p_x, imp_p_y, imp_p_z), PyKDL.Vector(imp_r_x, imp_r_y, imp_r_z))], [2], PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
            raise Exception("Could not change impedance of right lwr")
        if self.velma.waitForEffectorRight() != 0:
            raise Exception("Could not change impedance of right lwr")
        rospy.sleep(1)

    def setLeftLWRImpedance(self, imp_p_x, imp_p_y, imp_p_z, imp_r_x, imp_r_y, imp_r_z):
        if not self.velma.moveCartImpLeft(None, None, None, None, [PyKDL.Wrench(PyKDL.Vector(imp_p_x, imp_p_y, imp_p_z), PyKDL.Vector(imp_r_x, imp_r_y, imp_r_z))], [2], PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
            raise Exception("Could not change impedance of left lwr")
        if self.velma.waitForEffectorLeft() != 0:
            raise Exception("Could not change impedance of left lwr")
        rospy.sleep(1)

    def moveHeadTo(self, headPan, headTilt):
        desiredHeadState = (headPan, headTilt)
        self.velma.moveHead(desiredHeadState, 3.0, start_time=0.5)
        if self.velma.waitForHead() != 0:
            raise Exception("Moving head failed")
        rospy.sleep(0.5)

        if not isHeadConfigurationClose(self.velma.getHeadCurrentConfiguration(), desiredHeadState, 0.1):
            raise Exception("Head too far from destination")

    def moveTorso(self, angle):
        q = self.velma.getLastJointState()
        q[1]['torso_0_joint'] = angle
        self.velma.moveJoint(q[1], 15.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
        error = self.velma.waitForJoint()
        if error != 0:
            print "The action should have ended without error, but the error code is", error
            exitError(6)


    def lookForObject(self):
        self.moveHeadTo(0.0, 0.0)
        self.moveHeadTo(0.0, 0.9)
        self.moveHeadTo(1.3, 0.9)
        self.moveHeadTo(-1.3, 0.9)
        self.moveHeadTo(0.0, 0.0)
        self.moveTorso(0.7)
        self.moveHeadTo(1.0, 0.7)
        self.moveTorso(-0.7)
        self.moveHeadTo(0.0, 0.7)
        self.moveTorso(0.0)
        self.moveHeadTo(0.0, 0.0)

        octomap = self.octomapListener.getOctomap(timeout_s=5.0)
        self.trajectoryPlanner.processWorld(octomap)

    def moveInJointImpedanceModeWithPlan(self, desiredJointState):
        goalConstraint = qMapToConstraints(desiredJointState, 0.01, group=self.velma.getJointGroup("impedance_joints"))
        for i in range(5):
            rospy.sleep(0.5)
            lastJointState = self.velma.getLastJointState()
            print("Planning try no. {}".format(i))
            trajectory = self.trajectoryPlanner.plan(lastJointState[1], [goalConstraint], "impedance_joints", max_velocity_scaling_factor=0.15, planner_id="RRTConnect")
            if trajectory is None:
                continue

            print("Found trajectory, executing...")
            if not self.velma.moveJointTraj(trajectory, start_time=0.5):
                raise Exception("Could not move velma")
            if self.velma.waitForJoint() == 0:
                break
            else:
                print("Trajectory could not be completed")
                continue

        print("Trajectory executed")
        rospy.sleep(0.5)
        lastJointState = self.velma.getLastJointState()
        print("Trajectory executed dsds ")
        if not isConfigurationClose(desiredJointState, lastJointState[1]):
            raise Exception("Could not move joints to desired state")
        print("Trajectory executed xxx")

    def moveToInteractiveCursor(self):
        T_Wo_test = self.velma.getTf("Wo", "example_frame")
        if not self.velma.moveCartImpLeftCurrentPos(start_time=0.2):
            exitError(8)
        if self.velma.waitForEffectorLeft() != 0:
            exitError(9)
        if not self.velma.moveCartImpLeft([T_Wo_test], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
            exitError(8)
        if self.velma.waitForEffectorLeft() != 0:
            exitError(9)
        rospy.sleep(0.5)

    def openLeftHand(self):
        dest_q = [0.80, 0.80, 0.7, 0]
        self.velma.moveHandLeft(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
        if self.velma.waitForHandLeft() != 0:
            raise Exception("Hand error")
        rospy.sleep(0.5)
        if not isHandConfigurationClose(self.velma.getHandLeftCurrentConfiguration(), dest_q):
            raise Exception("Hand configuration")

    def openRightHand(self):
        dest_q = [0.80, 0.80, 0.7, 0]
        self.velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
        if self.velma.waitForHandRight() != 0:
            raise Exception("Right hand error")
        rospy.sleep(0.5)
        if not isHandConfigurationClose(self.velma.getHandRightCurrentConfiguration(), dest_q):
            raise Exception("Right hand configuration")

    def prepareForCatch(self):
        T_B_Wr = self.velma.getTf("B", "Wl")
        T_Wr_Gr = self.velma.getTf("Wl", "Gl")
        if not self.velma.moveCartImpLeft([T_B_Wr*T_Wr_Gr], [0.1], [T_Wr_Gr], [0.1], None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
            exitError(18)
        if self.velma.waitForEffectorLeft() != 0:
            exitError(19)
        rospy.sleep(0.5)

    def moveLeftRelativeToInCartImpMode(self, tf, x, y, z, angle, tol=10):
        (setpointX, setpointY, setpointZ, setpointYaw) = self.resolveRelativePosition(tf, x, y, z)
        targetFrame = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, setpointYaw+angle), PyKDL.Vector(setpointX, setpointY, setpointZ))
        result = self.moveLeftInCartesianImpedanceMode(targetFrame, tol)
        return result

    def moveLeftInCartesianImpedanceMode(self, T_B_dest, tol=10):    
        if not self.velma.moveCartImpLeft([T_B_dest], [10.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.1, path_tol=PyKDL.Twist(PyKDL.Vector(tol, tol, tol), PyKDL.Vector(tol, tol, tol))):
            raise Exception("Could not move in cartesian impedance mode")
        if self.velma.waitForEffectorLeft() != 0:
            if not self.velma.moveCartImpLeftCurrentPos(start_time=0.01):
                raise Exception("Could not make it to given position")
            return False
        else:
            return True

    def moveMobileBase(self, x_vel, y_vel, theta_vel, time):
        publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        msg = Twist()
        msg.linear.x = x_vel
        msg.linear.y = y_vel
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = theta_vel
        startTime = rospy.get_rostime()
        while rospy.get_rostime() - startTime < rospy.Duration.from_sec(time):
            publisher.publish(msg)

        stopMsg = Twist()
        publisher.publish(stopMsg)

class SetBaseDestination(smach_rcprg.TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface):
        smach_rcprg.TaskER.BlockingState.__init__(self, input_keys=['dest_name'], output_keys=['goal_pose'],
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])

        self.conversation_interface = conversation_interface

        self.description = u'Znajduję cel ruchu'

    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        #self.conversation_interface.addSpeakSentence( u'Zakończyłem zadanie' )
        self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe Ustalam gdzie jest miejsce przedmiotu' )
        if isinstance(userdata.dest_name, str):
            dest_name = userdata.dest_name.decode('utf-8')
        dest_name = userdata.dest_name.encode('utf-8').decode('utf-8')
        userdata.goal_pose = navigation.PoseDescription({'place_name':unicode(dest_name)})

        if self.__shutdown__:
            return 'shutdown'
        return 'ok'

class PrepareToMoveBase(smach_rcprg.TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface, velma_task_executor):
        smach_rcprg.TaskER.BlockingState.__init__(self, input_keys=[], output_keys=[],
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])

        self.conversation_interface = conversation_interface

        self.description = u'Przygotowuję się do ruchu bazy'
        self.velma_task_executor = velma_task_executor
    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        #self.conversation_interface.addSpeakSentence( u'Zakończyłem zadanie' )
        self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe Przygotowuję się do ruchu bazy' )
        print("Reset robot position")
        self.velma_task_executor.moveToStartingPosition()

        print("Hiding both hands")
        self.velma_task_executor.hideHands()

        if self.__shutdown__:
            return 'shutdown'
        return 'ok'

class PrepareToGrip(smach_rcprg.TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface, velma_task_executor):
        smach_rcprg.TaskER.BlockingState.__init__(self, input_keys=['object_container'], output_keys=['object_container_grab_pose'],
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])

        self.conversation_interface = conversation_interface

        self.description = u'Przygotowuję się do chwytu'
        self.velma_task_executor = velma_task_executor
    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        #self.conversation_interface.addSpeakSentence( u'Zakończyłem zadanie' )
        self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe Przygotowuję się do chwytu' )
        print("Preparing for grip")
        userdata.object_container_grab_pose = self.velma_task_executor.prepareForGrip(userdata.object_container)

        if self.__shutdown__:
            return 'shutdown'
        return 'ok'

class PrepareToGrip(smach_rcprg.TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface, velma_task_executor):
        smach_rcprg.TaskER.BlockingState.__init__(self, input_keys=['object_container'], output_keys=['object_container_grab_pose'],
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])

        self.conversation_interface = conversation_interface

        self.description = u'Przygotowuję się do chwytu'
        self.velma_task_executor = velma_task_executor
    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        #self.conversation_interface.addSpeakSentence( u'Zakończyłem zadanie' )
        self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe Przygotowuję się do chwytu' )
        print("Preparing for grip")
        userdata.object_container_grab_pose = self.velma_task_executor.prepareForGrip(userdata.object_container)

        if self.__shutdown__:
            return 'shutdown'
        return 'ok'

class OpenDoor(smach_rcprg.TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface, velma_task_executor):
        smach_rcprg.TaskER.BlockingState.__init__(self, input_keys=['object_container_grab_pose'], output_keys=[],
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])

        self.conversation_interface = conversation_interface

        self.description = u'Otwieram drzwi'
        self.velma_task_executor = velma_task_executor
    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        #self.conversation_interface.addSpeakSentence( u'Zakończyłem zadanie' )
        self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe Otwieram drzwi' )
        
        print("Moving closer to cabinet")
        self.velma_task_executor.moveRelativeToInCartImpMode(userdata.object_container_grab_pose, 0.40, -0.2, 0.09, 0.0)

        print("Finding the door")
        self.velma_task_executor.setRightLWRImpedance(500, 500, 900, 1000, 1000, 200)
        self.velma_task_executor.moveRelativeToInCartImpMode(userdata.object_container_grab_pose, 0.25, -0.2, 0.09, 0.0)

        print("Finding the door handle")
        self.velma_task_executor.moveRelativeToInCartImpMode(userdata.object_container_grab_pose, 0.30, -0.40, 0.09, 0.0)

        print("Open a lil bit")
        self.velma_task_executor.setRightLWRImpedance(200, 200, 900, 1000, 1000, 200)
        self.velma_task_executor.moveRelativeToInCartImpMode(userdata.object_container_grab_pose, 0.50, -0.40, 0.09, 0.0)

        print("Open a lil bit more")
        self.velma_task_executor.setRightLWRImpedance(300, 300, 900, 200, 200, 200)
        self.velma_task_executor.moveRelativeToInCartImpMode(userdata.object_container_grab_pose, 0.70, 0.1, 0.09, 0.0)

        if self.__shutdown__:
            return 'shutdown'
        return 'ok'

class CorrectBasePose(smach_rcprg.TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface, velma_task_executor):
        smach_rcprg.TaskER.BlockingState.__init__(self, input_keys=[], output_keys=[],
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])

        self.conversation_interface = conversation_interface

        self.description = u'Poprawiam pozycję bazy'
        self.velma_task_executor = velma_task_executor
    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        #self.conversation_interface.addSpeakSentence( u'Zakończyłem zadanie' )
        self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe Poprawiam pozycję bazy' )
        
        self.velma_task_executor.moveToStartingPosition()
        self.velma_task_executor.moveMobileBase(0.0, 0.2, 0.0, 1.0)

        if self.__shutdown__:
            return 'shutdown'
        return 'ok'

class LookForObject(smach_rcprg.TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface, velma_task_executor):
        smach_rcprg.TaskER.BlockingState.__init__(self, input_keys=[], output_keys=[],
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])

        self.conversation_interface = conversation_interface

        self.description = u'Szukam obiektu'
        self.velma_task_executor = velma_task_executor
    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        #self.conversation_interface.addSpeakSentence( u'Zakończyłem zadanie' )
        self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe Szukam obiektu' )
        
        
        print("Looking for object")
        self.velma_task_executor.lookForObject()

        if self.__shutdown__:
            return 'shutdown'
        return 'ok'

class ApproachObject(smach_rcprg.TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface, velma_task_executor):
        smach_rcprg.TaskER.BlockingState.__init__(self, input_keys=[], output_keys=[],
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])

        self.conversation_interface = conversation_interface

        self.description = u'Podchodzę ramieniem do obiektu'
        self.velma_task_executor = velma_task_executor
    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        #self.conversation_interface.addSpeakSentence( u'Zakończyłem zadanie' )
        self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe Podchodzę ramieniem do obiektu' )
        
        
        self.velma_task_executor.hideHands()
        print("Moving to grabbing position with planner")
        self.velma_task_executor.setJointImpedanceMode()
        desiredJointState = copy.deepcopy(self.velma_task_executor.catchingJointState)
        self.velma_task_executor.moveInJointImpedanceModeWithPlan(desiredJointState)

        if self.__shutdown__:
            return 'shutdown'
        return 'ok'

class TakeOutObject(smach_rcprg.TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface, velma_task_executor, marker_publisher):
        smach_rcprg.TaskER.BlockingState.__init__(self, input_keys=[], output_keys=[],
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])

        self.conversation_interface = conversation_interface

        self.description = u'Wyjmuję obiekt'
        self.velma_task_executor = velma_task_executor
        self.marker_publisher = marker_publisher
    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        #self.conversation_interface.addSpeakSentence( u'Zakończyłem zadanie' )
        self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe Wyjmuję obiekt' )
        
        print("Grabbing")
        print("Set cimp mode for right lwr")
        if not self.velma_task_executor.velma.moveCartImpRightCurrentPos(start_time=0.2):
          raise Exception("Could not set CartImp mode for right lwr")

        if self.velma_task_executor.velma.waitForEffectorRight() != 0:
          raise Exception("Right effector error")

        self.velma_task_executor.setRightLWRImpedance(1000, 1000, 1000, 1000, 1000, 1000)

        objectTF = self.velma_task_executor.velma.getTf("B", "object")
        self.velma_task_executor.moveRelativeToInCartImpMode(objectTF, -0.5, 0.0, 0.08, 3.14)

        self.velma_task_executor.hideHands()
        print("Open hand") # CZEKAJ 
        self.velma_task_executor.openRightHand()
        rospy.sleep(1.0)

        self.velma_task_executor.setRightLWRImpedance(300, 1000, 1000, 2000, 1000, 1000)
        self.velma_task_executor.moveRelativeToInCartImpMode(objectTF, -0.20, 0.0, 0.08, 3.14)

        print("Add collision")
        marker_publisher.start()

        print("Close hand on object")
        handDesiredState = dest_q = [74.0/180.0*math.pi, 74.0/180.0*math.pi, 74.0/180.0*math.pi, 0]
        self.velma_task_executor.velma.moveHandRight(handDesiredState, [1,1,1,1], [1000, 1000, 1000, 1000], 1000, hold=True)
        if self.velma_task_executor.velma.waitForHandRight() != 0:
          raise Exception("Could not catch with right hand")

        objectTF = self.velma_task_executor.velma.getTf("B", "object")
        print("lift")
        self.velma_task_executor.moveRelativeToInCartImpMode(objectTF, -0.23, 0.00, 0.15, 3.14)
           
        print("pull hand back")
        self.velma_task_executor.moveRelativeToInCartImpMode(objectTF, -0.65, 0.0, 0.15, 3.14)
        

        if self.__shutdown__:
            return 'shutdown'
        return 'ok'


class PutDownObject(smach_rcprg.TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface, velma_task_executor, marker_publisher):
        smach_rcprg.TaskER.BlockingState.__init__(self, input_keys=[], output_keys=[],
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])

        self.conversation_interface = conversation_interface

        self.description = u'Odstawiam obiekt'
        self.velma_task_executor = velma_task_executor
        self.marker_publisher = marker_publisher
    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        #self.conversation_interface.addSpeakSentence( u'Zakończyłem zadanie' )
        self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe Odstawiam obiekt' )

        print("Set cimp mode for right lwr")
        if not self.velma_task_executor.velma.moveCartImpRightCurrentPos(start_time=0.2):
          raise Exception("Could not set CartImp mode for right lwr")

        if self.velma_task_executor.velma.waitForEffectorRight() != 0:
          raise Exception("Right effector error")

        print "Moving the right tool and equilibrium pose from 'wrist' to 'grip' frame..."
        T_B_Wl = self.velma_task_executor.velma.getTf("B", "Wr")
        T_Wl_Gl = self.velma_task_executor.velma.getTf("Wr", "Gr")
        if not self.velma_task_executor.velma.moveCartImpRight([T_B_Wl*T_Wl_Gl], [0.1], [T_Wl_Gl], [0.1], None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
            exitError(18)
        # if self.velma_task_executor.velma.waitForEffectorRight() != 0:
        #     exitError(19)
        print "The right tool is now in 'grip' pose"
        rospy.sleep(0.5)

        tableTF = self.velma_task_executor.velma.getTf("B", "table")
        x = tableTF.p[0]
        y = tableTF.p[1]
        z = tableTF.p[2]
        
        rot = PyKDL.Rotation.RPY(0.0, 1.54, 0.0)
        BT = PyKDL.Frame(rot, PyKDL.Vector(x-0.20, y, z+1.2))

        
        print("Move hand over the table")
        tol = 10.0
        self.velma_task_executor.setRightLWRImpedance(1000, 1000, 200, 2000, 1000, 1000)
        if not self.velma_task_executor.velma.moveCartImpRight([BT], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.1, path_tol=PyKDL.Twist(PyKDL.Vector(tol, tol, tol), PyKDL.Vector(tol, 0.5, tol))):
            raise Exception("Could not move in cartesian impedance mode")
        if self.velma_task_executor.velma.waitForEffectorRight() != 0:
            if not self.velma_task_executor.velma.moveCartImpRightCurrentPos(start_time=0.01):
                raise Exception("Could not make it to given position")



        print("Put can on the table")
        BT = PyKDL.Frame(rot, PyKDL.Vector(x-0.20, y, z+0.7))
        tol = 10.0
        self.velma_task_executor.setRightLWRImpedance(200, 200, 500, 1000, 1000, 1000)
        if not self.velma_task_executor.velma.moveCartImpRight([BT], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.1, path_tol=PyKDL.Twist(PyKDL.Vector(tol, tol, tol), PyKDL.Vector(tol, 0.5, tol))):
            raise Exception("Could not move in cartesian impedance mode")
        if self.velma_task_executor.velma.waitForEffectorRight() != 0:
            if not self.velma_task_executor.velma.moveCartImpRightCurrentPos(start_time=0.01):
                raise Exception("Could not make it to given position")

        

        print("open hand")
        dest_q = [0.0, 0.0, 0.0, 0]
        self.velma_task_executor.velma.moveHandRight(dest_q, [10,10,10,10], [2000, 2000, 2000, 2000], 1000, hold=True)
        if self.velma_task_executor.velma.waitForHandRight() != 0:
            raise Exception("Hand error")
            rospy.sleep(0.5)

        print("pull back")
        BT = PyKDL.Frame(rot, PyKDL.Vector(x-0.30, y, z+0.9))
        tol = 10.0
        self.velma_task_executor.setRightLWRImpedance(200, 200, 200, 1000, 1000, 1000)
        if not self.velma_task_executor.velma.moveCartImpRight([BT], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.1, path_tol=PyKDL.Twist(PyKDL.Vector(tol, tol, tol), PyKDL.Vector(tol, 0.5, tol))):
            raise Exception("Could not move in cartesian impedance mode")
        if self.velma_task_executor.velma.waitForEffectorRight() != 0:
            if not self.velma_task_executor.velma.moveCartImpRightCurrentPos(start_time=0.01):
                raise Exception("Could not make it to given position")
        
        self.velma_task_executor.moveMobileBase(-0.2, 0.0, 0.0, 1.0)
        self.velma_task_executor.hideHands()
        self.velma_task_executor.moveToStartingPosition()
        self.marker_publisher.stop()

        if self.__shutdown__:
            return 'shutdown'
        return 'ok'



def GetJar(smach_rcprg.StateMachine):
    def __init__(self, sim_mode, conversation_interface, kb_places):
        smach_rcprg.StateMachine.__init__(self, input_keys=['object_container','bring_destination','end_pose','susp_data'], output_keys=['susp_data'],
                                        outcomes=['PREEMPTED',
                                                    'FAILED',
                                                    'FINISHED', 'shutdown'])
        self.userdata.max_lin_vel = 0.2
        self.userdata.max_lin_accel = 0.5
        # TODO: use knowledge base for this:
        self.userdata.default_height = 0.2
        self.userdata.lowest_height = 0.0

        self.description = u'Podaję rzecz'
        worker = VelmaTaskExecutor()
        object1 = AttachedCollisionObject()
        object1.link_name = "left_HandGripLink"
        object1.object.header.frame_id = "left_HandGripLink"
        object1.object.id = "object1"
        object1_prim = SolidPrimitive()
        object1_prim.type = SolidPrimitive.CYLINDER
        object1_prim.dimensions=[None, None]    # set initial size of the list to 2
        object1_prim.dimensions[SolidPrimitive.CYLINDER_HEIGHT] = 0.25
        object1_prim.dimensions[SolidPrimitive.CYLINDER_RADIUS] = 0.06
        object1_pose = pm.toMsg(PyKDL.Frame(PyKDL.Rotation.RotY(math.pi/2)))
        object1.object.primitives.append(object1_prim)
        object1.object.primitive_poses.append(object1_pose)
        object1.object.operation = CollisionObject.ADD
        object1.touch_links = [
            'right_FtSensorLink',
            'right_HandGripLink',
            'right_gripper_mount_link',
            'right_HandPalmLink',
            'right_HandFingerOneKnuckleOneLink',
            'right_HandFingerOneKnuckleTwoLink',
            'right_HandFingerOneKnuckleThreeLink',
            'right_HandFingerTwoKnuckleOneLink',
            'right_HandFingerTwoKnuckleTwoLink',
            'right_HandFingerTwoKnuckleThreeLink',
            'right_HandFingerThreeKnuckleOneLink',
            'right_HandFingerThreeKnuckleTwoLink',
            'right_HandFingerThreeKnuckleThreeLink']
        marker_publisher = MarkerPublisherThread(object1)


        # self.userdata.end_pose = 'init_velma_pose'

        with self:
            smach_rcprg.StateMachine.add('SetContainerDestination', SetBaseDestination(sim_mode, conversation_interface),
                                    transitions={'ok':'SetNavParams', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'dest_name':'object_container'})

            smach_rcprg.StateMachine.add('SetNavParams', navigation.SetNavParams(sim_mode),
                                    transitions={'ok':'PrepareToMoveBase', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'max_lin_vel_in':'max_lin_vel', 'max_lin_accel_in':'max_lin_accel'})

            smach_rcprg.StateMachine.add('PrepareToMoveBase', PrepareToMoveBase(sim_mode, conversation_interface, worker),
                                    transitions={'ok':'SetNavParams', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={})

            smach_rcprg.StateMachine.add('MoveToCabinet', navigation.MoveToComplex(sim_mode, conversation_interface, kb_places),
                                    transitions={'FINISHED':'PrepareToGrip', 'PREEMPTED':'PREEMPTED', 'FAILED': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'goal':'goal_pose', 'susp_data':'susp_data'})

            smach_rcprg.StateMachine.add('PrepareToGrip', PrepareToGrip(sim_mode, conversation_interface, worker),
                                    transitions={'ok':'OpenDoor', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={})

            smach_rcprg.StateMachine.add('OpenDoor', OpenDoor(sim_mode, conversation_interface, worker),
                                    transitions={'ok':'CorrectBasePose', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'object_container_pose':'object_container_pose'})

            smach_rcprg.StateMachine.add('CorrectBasePose', CorrectBasePose(sim_mode, conversation_interface, worker),
                                    transitions={'ok':'LookForObject', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={})

            smach_rcprg.StateMachine.add('LookForObject', LookForObject(sim_mode, conversation_interface, worker),
                                    transitions={'ok':'ApproachObject', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={})

            smach_rcprg.StateMachine.add('ApproachObject', ApproachObject(sim_mode, conversation_interface, worker),
                                    transitions={'ok':'TakeOutObject', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={})

            smach_rcprg.StateMachine.add('TakeOutObject', TakeOutObject(sim_mode, conversation_interface, worker, marker_publisher),
                                    transitions={'ok':'SetBringDestination', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={})

            smach_rcprg.StateMachine.add('SetBringDestination', SetBaseDestination(sim_mode, conversation_interface),
                                    transitions={'ok':'MoveToBringDestination', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'dest_name':'bring_destination'})


            smach_rcprg.StateMachine.add('MoveToBringDestination', navigation.MoveToComplex(sim_mode, conversation_interface, kb_places),
                                    transitions={'FINISHED':'PutDownObject', 'PREEMPTED':'PREEMPTED', 'FAILED': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'goal':'goal_pose', 'susp_data':'susp_data'})


            smach_rcprg.StateMachine.add('PutDownObject', PutDownObject(sim_mode, conversation_interface, worker, marker_publisher),
                                    transitions={'ok':'SetEndDestination', 'preemption':'PREEMPTED', 'error':'FAILED'
                                    ,'shutdown':'shutdown', },
                                    remapping={})

            smach_rcprg.StateMachine.add('SetEndDestination', SetBaseDestination(sim_mode, conversation_interface),
                                    transitions={'ok':'MoveToEnd', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'dest_name':'end_pose'})

            smach_rcprg.StateMachine.add('MoveToEnd', navigation.MoveToComplex(sim_mode, conversation_interface, kb_places),
                                    transitions={'FINISHED':'FINISHED', 'PREEMPTED':'PREEMPTED', 'FAILED': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'goal':'end_pose', 'susp_data':'susp_data'})
