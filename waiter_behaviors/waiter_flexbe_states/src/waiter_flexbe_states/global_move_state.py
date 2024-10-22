#!/usr/bin/env python3
import time
import tf
import rospy
import actionlib
import numpy as np

from random import seed, random
from flexbe_core import EventState, Logger

from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3

from std_srvs.srv import SetBool

from mir_actions.msg import MirMoveBaseAction, MirMoveBaseGoal
from waiter_msgs.msg import WaiterStatus, SvmData

import waiter_flexbe_states.sound_lib as sound_lib

class GlobalMoveState(EventState):
    def __init__(self, prefix="/hook/camera", max_retry=5, tolerance=1.7):
        super(GlobalMoveState, self).__init__(outcomes = ['abort', 'home', 'has_local', 'retry', 'wait', 'error', 'success', 'failed'],
                                                    input_keys=['global_pos'],
                                                    output_keys=['global_pos'])

        self.move_base_client = actionlib.SimpleActionClient("/move_base", MirMoveBaseAction)

        self.abort_srv = rospy.ServiceProxy(prefix+"/behavior_service/set_abort", SetBool)
        self.arm_srv = rospy.ServiceProxy(prefix+"/behavior_service/set_arm", SetBool)

        self.tolerance = tolerance
        self.max_retry = max_retry
        self.retry = 0
        self.status = WaiterStatus()
        seed(time.time)

        rospy.Subscriber(prefix+"/behavior_state/status", WaiterStatus, self.status_cb)

    def on_enter(self, userdata):
        print("enter with retry = %s"%self.retry)
        if(self.retry == 0):  # Fist move need to dock out
            goal = MirMoveBaseGoal()
            goal.move_task = 2  # reletive move
            goal.max_linear_speed = 0.3
            goal.collision_detection = True
            goal.target_pose.header.frame_id = "base_link"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = -0.5
            goal.target_pose.pose.orientation.w = 1
            goal.clear_costmaps = True
            try:
                self.move_base_client.send_goal(goal)
                time.sleep(1.5)
            except Exception as e:
                Logger.logwarn('Failed to send the command:\n%s' % str(e))

        pos = userdata.global_pos
        goal = MirMoveBaseGoal()
        goal.move_task = 1  # global_move
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = pos.x
        goal.target_pose.pose.position.y = pos.y
        goal.target_pose.pose.orientation.w = 1
        self.arm_srv(False)
        #time.sleep(0.5)
        #self.arm_srv(True)
        try:
            self.move_base_client.send_goal(goal)
        except Exception as e:
            Logger.logwarn('Failed to send the command:\n%s' % str(e))

    def execute(self, userdata):
        self.current_time = time.time()
        if(self.status.abort_command.data):
            self.retry = 0
            return 'abort'

        elif(self.status.has_local.data):
            # preempt move_base
            self.retry = 1
            return 'has_local'

        # Check if the action has been finished
        elif(self.move_base_client.get_result() is not None):
            result = self.move_base_client.get_result()
            print("move_base result is %s"%result.end_state)

            # Reach goal successfully
            if(result.end_state==0):
                print("reach goal but no target found")
            # Error from navigation
            else:
                sound_lib.avoid()
                print("navigation error")

            self.retry += 1
            if((self.retry > self.max_retry) and self.status.has_global.data):
                self.retry = 0 
                if(self.check_distance() < self.tolerance):
                    sound_lib.thanks()
                    return 'success'
                else:
                    sound_lib.failed()
                    return 'failed'

            elif((self.retry > self.max_retry) and not self.status.has_global.data):
                return 'error'

            elif((self.retry < self.max_retry) and self.status.has_global.data):
                userdata.global_pos = self.status.global_position
                return 'retry'

            elif((self.retry < self.max_retry) and not self.status.has_global.data):
                self.start_time = time.time()
                return 'wait'


    def status_cb(self, status):
        self.status = status


    def check_distance(self):
        x = self.status.global_position.x
        y = self.status.global_position.y

        target = np.array([x, y, 0])

        base_link = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0))
        base_link = base_link.transform.translation
        base_link = np.array([base_link.x, base_link.y, 0])

        v = target - base_link
        mag = np.linalg.norm(v)
        print(f'distance : {mag}')
        return mag