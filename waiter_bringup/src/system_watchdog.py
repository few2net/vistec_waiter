#!/usr/bin/env python3
"""
    This node used for behavior state machine.
"""

import rospy
import tf2_ros

from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
from waiter_msgs.msg import WaiterStatus, SvmData

from std_srvs.srv import SetBool


class SystemWatchdog:
    def __init__(self):
        rospy.init_node('system_watchdog_node')

        # Set up node parameters
        prefix = rospy.get_param('~prefix', "")

        # Set up node attributes
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.abort_command = False
        self.home_command = False
        self.serve_command = False
        self.svm_trained = False
        self.has_global = False
        self.global_pos = Vector3(-1,-1,-1)
        self.has_local = False
        self.local_center = Vector3(-1,-1,-1)
        self.local_pos = Vector3(-1,-1,-1)
        self.arm_control = False

        # Set up publish topics
        self.status_pub = rospy.Publisher(prefix+"/behavior_state/status", WaiterStatus, queue_size=10)

        # Set up service servers
        self.abort_srv = rospy.Service(prefix+"/behavior_service/set_abort", SetBool, self.set_abort)
        self.home_srv = rospy.Service(prefix+"/behavior_service/set_home", SetBool, self.set_home)
        self.serve_srv = rospy.Service(prefix+"/behavior_service/set_serve", SetBool, self.set_serve)
        self.arm_srv = rospy.Service(prefix+"/behavior_service/set_arm", SetBool, self.set_arm)

        # Set up subscribe topics
        rospy.Subscriber(prefix+"/target/global_position", Vector3, self.global_pos_cb)
        rospy.Subscriber(prefix+"/target/center", Vector3, self.center_cb)
        rospy.Subscriber(prefix+"/svm_data", SvmData, self.target_state_cb)

        # Run
        rospy.Timer(rospy.Duration(0.1), self.publish_status)

    def set_abort(self, data):
        self.abort_command = data.data
        return [True, '']

    def set_home(self, data):
        self.home_command = data.data
        return [True, '']

    def set_serve(self, data):
        self.serve_command = data.data
        return [True, '']

    def target_state_cb(self, svm_data):
        self.svm_trained = False if(len(svm_data.y.data)==0) else True

    def global_pos_cb(self, data):
        self.global_pos = data
        self.has_global = True if self.global_pos.x != -1 else False

    def center_cb(self, data):
        self.local_center = data
        if(data.x == -1):
            self.has_local = False
            self.local_pos = Vector3(-1,-1,-1)
        elif(data.x >= 0):
            self.has_local = True
            try:
                target = self.tf_buffer.lookup_transform('map', 'target', rospy.Time(0))
                self.local_pos = target.transform.translation
            except:
                pass

    def set_arm(self, data):
        self.arm_control = data.data
        return [True, '']

    def publish_status(self, event):
        status_msg = WaiterStatus()
        status_msg.abort_command.data = self.abort_command
        status_msg.home_command.data = self.home_command
        status_msg.serve_command.data = self.serve_command
        status_msg.svm_trained.data = self.svm_trained
        status_msg.has_global.data = self.has_global
        status_msg.global_position = self.global_pos
        status_msg.has_local.data = self.has_local
        status_msg.local_center = self.local_center
        status_msg.local_position = self.local_pos
        status_msg.arm_control.data = self.arm_control
        self.status_pub.publish(status_msg)


if __name__ == '__main__':
    SystemWatchdog()
    rospy.spin()