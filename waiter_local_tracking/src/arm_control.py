#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import argparse
import time
import sys

from math import pi
from std_msgs.msg import Bool, Float64MultiArray
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from geometry_msgs.msg import Vector3, TransformStamped

import moveit_commander
import moveit_msgs.msg

from waiter_msgs.msg import WaiterStatus

from moveit_commander.conversions import pose_to_list
from tf.transformations import euler_from_quaternion, euler_matrix, quaternion_about_axis

class ArmController:
    def __init__(self):
        rospy.init_node('arm_controller', anonymous=True)
        self.prefix = rospy.get_param('~prefix', "")

        # Moveit initialize
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        #self.move_group.set_max_acceleration_scaling_factor(0.05)
        self.move_group.set_max_velocity_scaling_factor(0.03)
        #self.move_group.set_num_planning_attempts(5)
        

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)


        self.WIDTH = 640
        self.HEIGHT = 480
        self.mtx = []
        self.dist = []
        self.has_info = False
        self.target = np.array([self.WIDTH/2.0, self.HEIGHT/2.0])
        self.center = np.array([self.WIDTH/2.0, self.HEIGHT/2.0])
        self.is_home = False
        self.status = WaiterStatus()

        rospy.Subscriber(self.prefix+"/local_cam/color/camera_info", CameraInfo, self.build_camera_info)
        rospy.Subscriber("/behavior_state/status", WaiterStatus, self.status_cb)

        rospy.sleep(1)
        self.set_home()
        self.keep_tracking()

    def set_home(self):
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = 3.89
        joint_goal[1] = -2.94
        joint_goal[2] = 1.84
        joint_goal[3] = -2.09
        joint_goal[4] = -1.57
        joint_goal[5] = 0.00
        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()
        self.is_home = True

    def build_camera_info(self, msg):
        if(not self.has_info):
            self.mtx = np.array(msg.K).reshape([3, 3])
            self.dist = np.array(msg.D)
            self.has_info = True

    def keep_tracking(self):
        while(not rospy.is_shutdown()):
            if(self.status.arm_control.data):
                target_locked = self.target  # use static target for stable control 
                self.is_home = False
                if(self.target[0] == -1):
                    joint_goal = self.move_group.get_current_joint_values()
                    joint_goal[3] = -2.09
                    joint_goal[4] = -1.57
                    self.move_group.go(joint_goal, wait=True)
                    self.move_group.stop()

                else:
                    distance = self.target - self.center  # pixel
                    if(np.any(np.absolute(distance) > 50)):
                        joint_goal = self.move_group.get_current_joint_values()
                        joint_goal[3] = joint_goal[3] + np.arctan2(distance[1],self.mtx[1,1])  # y axis
                        joint_goal[4] = joint_goal[4] - np.arctan2(distance[0],self.mtx[0,0])  # x axis's cam is -x axis's tool0 
                        self.move_group.go(joint_goal, wait=True)
                        self.move_group.stop()
            elif(not self.status.arm_control.data and not self.is_home):
                self.set_home()

    def status_cb(self, status):
        self.status = status
        self.target = [self.status.local_center.x, self.status.local_center.y]


if __name__ == "__main__":
    node = ArmController()
    

