#!/usr/bin/env python3
import time
import numpy as np
from random import seed, random
import tf2_ros
import rospy
import actionlib
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3, PoseStamped

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient, ProxySubscriberCached, ProxyPublisher
from tf.transformations import quaternion_from_euler

from mir_actions.msg import MirMoveBaseAction, MirMoveBaseGoal
from std_srvs.srv import SetBool
from waiter_msgs.msg import WaiterStatus, SvmData

class LocalMoveState(EventState):
    def __init__(self, prefix="/hook/camera", max_retry=5, tolerance=1.3):
        super(LocalMoveState, self).__init__(outcomes = ['error' ,'abort', 'success'],
                                                    input_keys=['global_pos'])

        self.move_base_client = actionlib.SimpleActionClient("/move_base", MirMoveBaseAction)

        self.abort_srv = rospy.ServiceProxy(prefix+"/behavior_service/set_abort", SetBool)
        self.home_srv = rospy.ServiceProxy(prefix+"/behavior_service/set_home", SetBool)
        self.serve_srv = rospy.ServiceProxy(prefix+"/behavior_service/set_serve", SetBool)
        self.arm_srv = rospy.ServiceProxy(prefix+"/behavior_service/set_arm", SetBool)

        self.status = WaiterStatus()

        # It may happen that the action client fails to send the action goal.
        self.GOAL_OFFSET = 1.0
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.goal_num = 0
        self.tolerance = tolerance
        self.success = False
        self.max_retry = max_retry
        self.retry = 0
        self.switch_state = True
        seed(time.time)

        rospy.Subscriber(prefix+"/behavior_state/status", WaiterStatus, self.status_cb)
        rospy.Subscriber("/limit_switch", Bool, self.switch_cb)

    def on_enter(self, userdata):
        self.arm_srv(True)
        if(self.check_distance() < self.tolerance):
            self.success = True
        
        else:
            goal = MirMoveBaseGoal()
            goal.move_task = 1  # global_move
            goal.target_pose = self.calculate_goal()
 
            try:
                self.move_base_client.send_goal(goal)
            except Exception as e:
                Logger.logwarn('Failed to send the command:\n%s' % str(e))


    def execute(self, userdata):

        if(self.status.abort_command.data):
            return 'abort'

        elif(self.success):
            return 'success'
        
        # Check if the action has been finished
        elif(self.move_base_client.get_result() is not None):
            result = self.move_base_client.get_result()
            print(result.end_state)
            if(result.end_state==0):
                print("goal reached")
                return 'success'
            else:
                print("navigation error")
                self.serve_srv(False)
                return 'error'
        
        
    def status_cb(self, status):
        self.status = status

    def switch_cb(self, msg):
        self.switch_state = msg.data

    def check_distance(self):
        target = self.tf_buffer.lookup_transform('map', 'target', rospy.Time(0))
        target = target.transform.translation
        target = np.array([target.x, target.y, 0])

        base_link = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0))
        base_link = base_link.transform.translation
        base_link = np.array([base_link.x, base_link.y, 0])

        v = target - base_link
        mag = np.linalg.norm(v)
        print(f'distance : {mag}')
        return mag



    def calculate_goal(self):
        target = self.tf_buffer.lookup_transform('map', 'target', rospy.Time(0))
        target = target.transform.translation
        target = np.array([target.x, target.y, 0])

        base_link = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0))
        base_link = base_link.transform.translation
        base_link = np.array([base_link.x, base_link.y, 0])

        print(f'target is {target}')
        print(f'base is {base_link}')
        v = target - base_link
        mag = np.linalg.norm(v)
        unit_v = v / mag
        goal_trans = (unit_v * (mag-self.GOAL_OFFSET)) + base_link

        yaw = np.arctan2(unit_v[1],unit_v[0])
        qua = quaternion_from_euler(0,0,yaw+1.57)
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = goal_trans[0]
        goal.pose.position.y = goal_trans[1]
        goal.pose.position.z = 0
        goal.pose.orientation.x = qua[0]
        goal.pose.orientation.y = qua[1]
        goal.pose.orientation.z = qua[2]
        goal.pose.orientation.w = qua[3]
        print(f'local goal is : {goal}')
        return goal