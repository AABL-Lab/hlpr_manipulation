#!/usr/bin/env python

import sys
import os
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.srv
from moveit_msgs.srv import GetStateValidityRequest, GetStateValidity
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
import actionlib
import geometry_msgs.msg
import std_msgs.msg
import wpi_jaco_msgs.msg
import wpi_jaco_msgs.srv
import math
import numpy as np
from hlpr_manipulation_utils.arm_moveit2 import ArmMoveIt
from hlpr_manipulation_utils.manipulator import Gripper
import hlpr_manipulation_utils.transformations as Transform
from hlpr_manipulation_utils.msg import RLActionServerActionGoal, RLActionServerActionResult, RLActionServerActionFeedback
from hlpr_manipulation_utils.msg import RLActionServerAction
from geometry_msgs.msg import Pose, PoseStamped, Point
from ros

def QuaternionNorm(Q_raw):
    qx_temp,qy_temp,qz_temp,qw_temp = Q_raw[0:4]
    qnorm = math.sqrt(qx_temp*qx_temp + qy_temp*qy_temp + qz_temp*qz_temp + qw_temp*qw_temp)
    qx_ = qx_temp/qnorm
    qy_ = qy_temp/qnorm
    qz_ = qz_temp/qnorm
    qw_ = qw_temp/qnorm
    Q_normed_ = [qx_, qy_, qz_, qw_]
    return Q_normed_


def Quaternion2EulerXYZ(Q_raw):
    Q_normed = QuaternionNorm(Q_raw)
    qx_ = Q_normed[0]
    qy_ = Q_normed[1]
    qz_ = Q_normed[2]
    qw_ = Q_normed[3]

    tx_ = math.atan2((2 * qw_ * qx_ - 2 * qy_ * qz_), (qw_ * qw_ - qx_ * qx_ - qy_ * qy_ + qz_ * qz_))
    ty_ = math.asin(2 * qw_ * qy_ + 2 * qx_ * qz_)
    tz_ = math.atan2((2 * qw_ * qz_ - 2 * qx_ * qy_), (qw_ * qw_ + qx_ * qx_ - qy_ * qy_ - qz_ * qz_))
    EulerXYZ_ = [tx_,ty_,tz_]
    return EulerXYZ_


def EulerXYZ2Quaternion(EulerXYZ_):
    tx_, ty_, tz_ = EulerXYZ_[0:3]
    sx = math.sin(0.5 * tx_)
    cx = math.cos(0.5 * tx_)
    sy = math.sin(0.5 * ty_)
    cy = math.cos(0.5 * ty_)
    sz = math.sin(0.5 * tz_)
    cz = math.cos(0.5 * tz_)

    qx_ = sx * cy * cz + cx * sy * sz
    qy_ = -sx * cy * sz + cx * sy * cz
    qz_ = sx * sy * cz + cx * cy * sz
    qw_ = -sx * sy * sz + cx * cy * cz

    Q_ = [qx_, qy_, qz_, qw_]
    return Q_


def unitParser(unit_, pose_value_, relative_):
    """ Argument unit """
    global currentCartesianCommand

    position_ = pose_value_[:3]
    orientation_ = pose_value_[3:]

    for i in range(0,3):
        if relative_:
            position_[i] = pose_value_[i] + currentCartesianCommand[i]
        else:
            position_[i] = pose_value_[i]

    # print('pose_value_ in unitParser 1: {}'.format(pose_value_))  # debug

    if unit_ == 'mq':
        if relative_:
            orientation_XYZ = Quaternion2EulerXYZ(orientation_)
            orientation_xyz_list = [orientation_XYZ[i] + currentCartesianCommand[3+i] for i in range(0,3)]
            orientation_q = EulerXYZ2Quaternion(orientation_xyz_list)
        else:
            orientation_q = orientation_

        orientation_rad = Quaternion2EulerXYZ(orientation_q)
        orientation_deg = list(map(math.degrees, orientation_rad))

    elif unit_ == 'mdeg':
        if relative_:
            orientation_deg_list = list(map(math.degrees, currentCartesianCommand[3:]))
            orientation_deg = [orientation_[i] + orientation_deg_list[i] for i in range(0,3)]
        else:
            orientation_deg = orientation_

        orientation_rad = list(map(math.radians, orientation_deg))
        orientation_q = EulerXYZ2Quaternion(orientation_rad)

    elif unit_ == 'mrad':
        if relative_:
            orientation_rad_list =  currentCartesianCommand[3:]
            orientation_rad = [orientation_[i] + orientation_rad_list[i] for i in range(0,3)]
        else:
            orientation_rad = orientation_

        orientation_deg = list(map(math.degrees, orientation_rad))
        orientation_q = EulerXYZ2Quaternion(orientation_rad)

    else:
        raise Exception("Cartesian value have to be in unit: mq, mdeg or mrad")

    pose_mq_ = position_ + orientation_q
    pose_mdeg_ = position_ + orientation_deg
    pose_mrad_ = position_ + orientation_rad

    # print('pose_mq in unitParser 1: {}'.format(pose_mq_))  # debug

    return pose_mq_, pose_mdeg_, pose_mrad_

class RlActionServerclass:
  def __init__(self):
    _feedback = RLActionServerActionFeedback()
    _result = RLActionServerActionResult()

    self._name = 'rl_actions'
    self.server = actionlib.SimpleActionServer('rl_actions', RLActionServerAction, self.execute, auto_start=False)
    self.server.start()

    self.collison_service = rospy.ServiceProxy('/check_state_validity', GetStateValidity)

    self.arm = ArmMoveIt("j2s7s300_link_base")
    self.grip = Gripper()


  def execute(self, goal, root=None):
    """
      Given a message of type geometry_msgs/Pose , move arm to 
      that goal with collision checking. The goal must be similar to 
      the current position as this action server is meant for 
      real time reinforcement learning. The orientation given
      in the Pose msg is more or less ingored.

      Parameters
      ----------
      goal: geometry_msgs/Pose
      
      root : string, optional
            the root link (if none is provided, uses the planning frame)
    """
    r = rospy.Rate(.1)
    success = True
    self.server.set_succeeded()

    rospy.loginfo('%s: Checking collisions and executing at time %i' % 
                  (self._name, goal.order, rospy.Time.now()))

    goal_pose = np.array([goal.position.x, goal.position.y, goal.postion.z])
    goal_msg = PoseStamped()

    if root is None:
      goal_msg.header.frame_id=self.arm.get_planning_frame()
    else:
      goal_msg.header.frame_id=root

    goal_joints = self.arm.get_IK(goal)
    
    # check change in pose is small: 

    # get current joints in sorted order
    arm_curr_pose = self.arm.get_current_pose()
    curr_joints = np.array([arm_curr_pose[x] for x in sorted(arm_curr_pose)])
    curr_pose = np.array([self.arm.get_FK()[0].pose.position.x, self.arm.get_FK()[0].pose.position.y,
                          self.arm.get_FK()[0].pose.position.z])

    # check for current collision or goal collision:
    curr_robot_state = self.arm.state_from_joints(arm_curr_pose)

    eef_change = curr_pose - goal_pose
    joint_change = curr_joints - goal_joints           

    

