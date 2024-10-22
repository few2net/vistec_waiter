#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import argparse
import time
import sys

from math import pi
from std_msgs.msg import Bool, Float64MultiArray
from sensor_msgs.msg import Image, CompressedImage, CameraInfo, JointState
from geometry_msgs.msg import Vector3, TransformStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from waiter_msgs.msg import WaiterStatus
from controller_manager_msgs.srv import SwitchController


class ArmController:
    def __init__(self):
        rospy.init_node('arm_controller')
        rospy.on_shutdown(self.reset_controller)
        prefix = rospy.get_param('~prefix', "")

        rospy.wait_for_service("/controller_manager/switch_controller")
        self.switcher = rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)

        self.current_controller = "POS"  # "POS" or "VEL"
        self.state = "INIT"  # ["RESET", "IDLE", "TRACK", "LOST"]
        self.WIDTH = 640
        self.HEIGHT = 480
        self.time_to_reset = 2.0
        self.start_time = 0.0
        self.current_time = 0.0
        self.mtx = []
        self.dist = []
        self.has_info = False
        self.target = np.array([self.WIDTH/2.0, self.HEIGHT/2.0])
        self.center = np.array([self.WIDTH/2.0, self.HEIGHT/2.0])
        self.Kp = np.array([0.004, 0.004])
        self.is_home = False
        self.max_limit = (-1.46, 0.06)  # wrist_1, wrist_2
        self.min_limit = (-2.22, -3.08)  # wrist_1, wrist_2
        self.status = WaiterStatus()
        self.joint_states = JointState()

        rospy.Subscriber(prefix+"/local_cam/color/camera_info", CameraInfo, self.build_camera_info)
        rospy.Subscriber(prefix+"/behavior_state/status", WaiterStatus, self.status_cb)
        rospy.Subscriber("/joint_states", JointState, self.joint_cb)

        self.pos_controller = rospy.Publisher("/scaled_pos_joint_traj_controller/command", JointTrajectory, queue_size=10)
        self.vel_controller = rospy.Publisher("/joint_group_vel_controller/command", Float64MultiArray, queue_size=10)


        self.target_pub = rospy.Publisher('/test_pid/target', Vector3, queue_size=10)
        self.current_pub = rospy.Publisher('/test_pid/current', Vector3, queue_size=10)
        self.vel_pub = rospy.Publisher('/test_pid/velocity', Vector3, queue_size=10)

        rospy.sleep(1)
        self.keep_tracking()

    def start_pos_controller(self):
        print("POS controller ON!!!")
        self.switcher(["scaled_pos_joint_traj_controller"],["joint_group_vel_controller"],2,True,0)

    def start_vel_controller(self):
        print("VEL controller ON!!!")
        self.switcher(["joint_group_vel_controller"],["scaled_pos_joint_traj_controller"],2,True,0)

    def _set_home(self):
        print("reset.....")
        # scaled_pos_joint_traj_controller should be running
        self.start_pos_controller()
        rospy.sleep(3)
        joint_msg = JointTrajectory()
        joint_msg.joint_names = ["elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint",
                                "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        point = JointTrajectoryPoint()
        point.positions = [1.84,-2.94,3.89,-2.09,-1.57,0.00]
        point.time_from_start = rospy.Duration(3)
        joint_msg.points = [point]
        self.pos_controller.publish(joint_msg)
        rospy.sleep(3)
        self.start_vel_controller()
        rospy.sleep(1)

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
        while(not rospy.is_shutdown()):
            #print(self.state)
            if(self.status.arm_control.data):
                if(self.state == "RESET"):
                    self._set_home()
                    self.state = "IDLE"
                    self.target_pub.publish(0,0,0)
                    self.current_pub.publish(self.center[0],self.center[1],0)

                elif(self.state == "IDLE"):
                    self.target_pub.publish(0,0,0)
                    self.current_pub.publish(self.center[0],self.center[1],0)
                    if(not np.any(self.target == -1.0)):
                        self.state = "TRACK"

                elif(self.state == "TRACK"):
                    #print(f'target : {self.target}')
                    if(not np.any(self.target == -1.0)):
                        # Debugging
                        self.target_pub.publish(self.target[0],self.target[1],0)
                        self.current_pub.publish(self.center[0],self.center[1],0)
                        # PID
                        error = self.target - self.center
                        vel = self.Kp * error
                        #print(f'vel: {vel}')
                        # Limit joint
                        current_joint = np.array(self.joint_states.position[3:5])
                        mask_limit = np.logical_and(current_joint<self.max_limit, current_joint>self.min_limit)

                        vel = vel * mask_limit[::-1]

                        self.vel_pub.publish(vel[0],vel[1],0)
                        self._send_velocity([0, 0, 0, vel[1], -vel[0], 0])

                    else:
                        self.target_pub.publish(0,0,0)
                        self.current_pub.publish(self.center[0],self.center[1],0)
                        self.state = "LOST"
                        self._send_velocity([0, 0, 0, 0, 0, 0])
                        self.start_time = time.time()

                elif(self.state == "LOST"):
                    self.target_pub.publish(0,0,0)
                    self.current_pub.publish(self.center[0],self.center[1],0)
                    if(not np.any(self.target == -1.0)):
                        self.state = "TRACK"

                    else:
                        self.current_time = time.time()

                    if((self.current_time-self.start_time)>self.time_to_reset):
                        self.state = "RESET"
                        
            elif(not self.status.arm_control.data):
                if(self.state == "IDLE"):
                    continue

                else:
                    self.state = "IDLE"
                    self._set_home()


    def status_cb(self, status):
        self.status = status
        self.target = np.array([self.status.local_center.x, self.status.local_center.y])

    def joint_cb(self, msg):
        self.joint_states = msg


if __name__ == "__main__":
    node = ArmController()
    

