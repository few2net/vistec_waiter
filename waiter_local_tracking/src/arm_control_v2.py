#!/usr/bin/env python3
"""
    Don't forget to set the joint limits (ur_description/config/ur3e/joint_limits.yaml)
    according to these values:
        wrist_1 (tilt joint) -->  -120 to -80 degrees
        wrist_2 (pan_joint)  -->  -180 to 0 degrees
"""

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
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from waiter_msgs.msg import WaiterStatus
from controller_manager_msgs.srv import SwitchController


class ArmController:
    def __init__(self):
        rospy.init_node('arm_controller')
        rospy.on_shutdown(self.reset_controller)
        prefix = rospy.get_param('~prefix', "")

        self.switcher = rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)


        self.pos_controller = rospy.Publisher("/scaled_pos_joint_traj_controller/command", JointTrajectory, queue_size=10)
        self.vel_controller = rospy.Publisher("/joint_group_vel_controller/command", Float64MultiArray, queue_size=10)


        self.target_pub = rospy.Publisher('/test_pid/target', Vector3, queue_size=10)
        self.current_pub = rospy.Publisher('/test_pid/current', Vector3, queue_size=10)
        self.vel_pub = rospy.Publisher('/test_pid/velocity', Vector3, queue_size=10)

        self.current_controller = "POS"  # "POS" or "VEL"

        self.WIDTH = 640
        self.HEIGHT = 480
        self.reset_state = False
        self.time_to_reset = 2.0
        self.start_time = 0.0
        self.current_time = 0.0
        self.mtx = []
        self.dist = []
        self.has_info = False
        self.target = np.array([self.WIDTH/2.0, self.HEIGHT/2.0])
        self.center = np.array([self.WIDTH/2.0, self.HEIGHT/2.0])
        self.Kp = np.array([0.003, 0.001])
        self.is_home = False
        self.status = WaiterStatus()

        rospy.Subscriber(prefix+"/local_cam/color/camera_info", CameraInfo, self.build_camera_info)
        rospy.Subscriber(prefix+"/behavior_state/status", WaiterStatus, self.status_cb)

        rospy.sleep(1)
        self._set_home()
        self.start_vel_controller()
        self.keep_tracking()

    def start_pos_controller(self):
        self.switcher(["scaled_pos_joint_traj_controller"],["joint_group_vel_controller"],2,True,0)

    def start_vel_controller(self):
        self.switcher(["joint_group_vel_controller"],["scaled_pos_joint_traj_controller"],2,True,0)

    def _set_home(self):
        print("reset.....")
        # scaled_pos_joint_traj_controller should be running
        joint_msg = JointTrajectory()
        joint_msg.joint_names = ["elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint",
                                "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        point = JointTrajectoryPoint()
        point.positions = [1.84,-2.94,3.89,-2.09,-1.57,0.00]
        point.time_from_start = rospy.Duration(3)
        joint_msg.points = [point]
        self.pos_controller.publish(joint_msg)
        rospy.sleep(3)

    def _send_velocity(self, vel):
        # joint_group_vel_controller should be running
        vel_msg = Float64MultiArray()
        vel_msg.data = vel
        self.vel_controller.publish(vel_msg)

    def build_camera_info(self, msg):
        if(not self.has_info):
            self.mtx = np.array(msg.K).reshape([3, 3])
            self.dist = np.array(msg.D)
            self.has_info = True

    def reset_controller(self):
        self.start_pos_controller()

    def keep_tracking(self):
        print("start tracking")
        while(not rospy.is_shutdown()):
            if(self.status.arm_control.data):
                self.current_time = time.time()
                if(np.any(self.target == -1.0)):
                    print("no target")
                    print(f'target : {self.target}')
                    self._send_velocity([0, 0, 0, 0, 0, 0])
                    if(self.reset_state == False):
                        self.start_time = self.current_time
                        self.reset_state = True
                    elif(self.reset_state == True and (self.current_time-self.start_time)>self.time_to_reset):
                        self.reset_state = False
                        self.start_pos_controller()
                        self._set_home()
                        self.start_vel_controller()
                else:
                    self.reset_state = False
                    self.target_pub.publish(self.target[0],self.target[1],0)
                    self.current_pub.publish(self.center[0],self.center[1],0)
                    print(f'target : {self.target}')
                    error = self.target - self.center
                    #print(f'error : {error}')
                    vel = self.Kp * error
                    self.vel_pub.publish(vel[0],vel[1],0)
                    self._send_velocity([0, 0, 0, vel[1], -vel[0], 0])


            """
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
            """

    def status_cb(self, status):
        self.status = status
        self.target = np.array([self.status.local_center.x, self.status.local_center.y])


if __name__ == "__main__":
    node = ArmController()
    

