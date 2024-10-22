#!/usr/bin/env python3
import time
import tf
import rospy
import requests
import actionlib

from random import seed, random
from flexbe_core import EventState, Logger

from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3

from std_srvs.srv import SetBool

from mir_actions.msg import MirMoveBaseAction, MirMoveBaseGoal
from waiter_msgs.msg import WaiterStatus, SvmData

import waiter_flexbe_states.sound_lib as sound_lib


class HomeMoveState(EventState):
    
    def __init__(self, prefix="/hook/camera"):
        super(HomeMoveState, self).__init__(outcomes = ['success', 'abort'])

        self.abort_srv = rospy.ServiceProxy(prefix+"/behavior_service/set_abort", SetBool)
        self.home_srv = rospy.ServiceProxy(prefix+"/behavior_service/set_home", SetBool)
        self.serve_srv = rospy.ServiceProxy(prefix+"/behavior_service/set_serve", SetBool)

        # MIR api
        mir_ip = "http://192.168.12.20"
        api_version = "v2.0.0"
        self.header = {"Authorization":"Basic ZGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA=="}
        self.url = mir_ip + "/api/" + api_version + "/"
        self.dock_id = 0

    def on_enter(self, userdata):
        self.dock_id = self.dock_to_marker()["id"]

    def execute(self, userdata):
        state = self.get_mission_state(self.dock_id)
        if(state == "Done"):
            return 'success'

        elif(state == "Aborted"):
            return 'abort'

    def dock_to_marker(self):
        sound_lib.home()
        end_point = "mission_queue"
        data = {
                    "mission_id": "mirconst-guid-0000-0005-actionlist00",
                    "parameters": [
                        {
                            "id": "Marker",
                            "value": "5d5cf372-5907-11ec-bb21-0001299df266"
                        }
                    ]
                }
        response = requests.post(self.url+end_point, headers=self.header, json=data)
        return response.json()

    def get_mission_state(self, mission_id):
        end_point = "mission_queue/" + str(mission_id)
        response = requests.get(self.url+end_point, headers=self.header).json()
        return response["state"]