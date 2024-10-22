#!/usr/bin/env python3
import rospy
import time

from flexbe_core import EventState, Logger
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from geometry_msgs.msg import Vector3
from waiter_msgs.msg import WaiterStatus, SvmData

class LookingState(EventState):
    def __init__(self, prefix="/hook/camera", wait_duration=20):
        super(LookingState, self).__init__(outcomes = ['home', 'has_global', 'abort', 'clear', 'timeout'],
                                                output_keys=['global_pos'])

        self.abort_srv = rospy.ServiceProxy(prefix+"/behavior_service/set_abort", SetBool)
        self.home_srv = rospy.ServiceProxy(prefix+"/behavior_service/set_home", SetBool)
        self.serve_srv = rospy.ServiceProxy(prefix+"/behavior_service/set_serve", SetBool)

        self.status = WaiterStatus()

        self.start_time = 0
        self.current_time = 0
        self.wait_duration = wait_duration

        rospy.Subscriber(prefix+"/behavior_state/status", WaiterStatus, self.status_cb)

    def on_enter(self, userdata):
        self.start_time = time.time()

    def execute(self, userdata):
        self.current_time = time.time()
        if(self.status.home_command.data):
            self.start_time = 0
            self.current_time = 0
            return 'home'

        elif(self.status.abort_command.data):
            self.start_time = 0
            self.current_time = 0
            return 'abort'
        
        elif(self.status.has_global.data):
            userdata.global_pos = self.status.global_position
            self.start_time = 0
            self.current_time = 0
            return 'has_global'
        
        elif(not self.status.svm_trained.data):
            self.start_time = 0
            self.current_time = 0
            return 'clear'

        elif((self.current_time-self.start_time) > self.wait_duration):
            self.start_time = 0
            self.current_time = 0
            return 'timeout'

    def status_cb(self, status):
        self.status = status

