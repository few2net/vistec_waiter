#!/usr/bin/env python3
import rospy
import time
from std_msgs.msg import Bool
from flexbe_core import EventState, Logger
from std_srvs.srv import SetBool
from waiter_msgs.msg import WaiterStatus, SvmData

class IdleState(EventState):
    def __init__(self, prefix="/hook/camera"):
        super(IdleState, self).__init__(outcomes = ['home', 'look', 'abort'])

        self.abort_srv = rospy.ServiceProxy(prefix+"/behavior_service/set_abort", SetBool)
        self.home_srv = rospy.ServiceProxy(prefix+"/behavior_service/set_home", SetBool)
        self.serve_srv = rospy.ServiceProxy(prefix+"/behavior_service/set_serve", SetBool)
        self.arm_srv = rospy.ServiceProxy(prefix+"/behavior_service/set_arm", SetBool)
        self.status = WaiterStatus()

        rospy.Subscriber(prefix+"/behavior_state/status", WaiterStatus, self.status_cb)


    def on_enter(self, userdata):
        self.abort_srv(False)
        self.serve_srv(False)
        self.home_srv(False)
        self.arm_srv(False)
        time.sleep(1)

    def execute(self, userdata):

        if(self.status.home_command.data):
            return 'home'
        
        elif(self.status.abort_command.data):
            return 'abort'

        elif(self.status.svm_trained.data and self.status.serve_command.data):
            return 'look'

    def status_cb(self, status):
        self.status = status
