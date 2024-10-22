#!/usr/bin/env python3
import rospy
import time
from std_msgs.msg import Bool
from flexbe_core import EventState, Logger
from std_srvs.srv import SetBool
from waiter_msgs.msg import WaiterStatus, SvmData

import waiter_flexbe_states.sound_lib as sound_lib

class WaitState(EventState):
    def __init__(self, prefix="/hook/camera"):
        super(WaitState, self).__init__(outcomes = ['abort','timeout', 'success', 'failed'])

        #self.abort_srv = rospy.ServiceProxy(prefix+"/behavior_service/set_abort", SetBool)
        self.home_srv = rospy.ServiceProxy(prefix+"/behavior_service/set_home", SetBool)
        self.serve_srv = rospy.ServiceProxy(prefix+"/behavior_service/set_serve", SetBool)
        self.arm_srv = rospy.ServiceProxy(prefix+"/behavior_service/set_arm", SetBool)

        self.status = WaiterStatus()
        self.switch_state = True
        self.start_time = 0
        self.current_time = 0
        self.timeout = 3
        self.retry = 0
        self.max_retry = 5

        rospy.Subscriber(prefix+"/behavior_state/status", WaiterStatus, self.status_cb)
        rospy.Subscriber("/limit_switch", Bool, self.switch_cb)

    def on_enter(self, userdata):
        sound_lib.please()
        self.start_time = time.time()

    def execute(self, userdata):
        self.current_time = time.time()

        """
        elif(self.status.home_command.data):
            self.retry = 0
            self.home_srv(False)
            return 'home'
        """

        if(self.status.abort_command.data):
            self.retry = 0
            #self.abort_srv(False)
            return 'abort'

        elif(not self.switch_state):
            self.retry = 0
            self.serve_srv(False)
            sound_lib.thanks()
            return 'success'

        elif((self.current_time-self.start_time) > self.timeout):
            self.arm_srv(False)
            self.retry += 1

            if(self.retry > self.max_retry):
                self.retry = 0
                sound_lib.failed()
                return 'failed'
            else:
                return 'timeout'

    def status_cb(self, status):
        self.status = status

    def switch_cb(self, msg):
        self.switch_state = msg.data