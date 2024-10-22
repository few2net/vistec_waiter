#!/usr/bin/env python3
"""
    I cannot ssh to the Mir's PC, so I synced Mir2's PC clock to the Mir's PC
    by
    1. Sync the timedate: 
            ntpdate 192.168.12.20
    
    2. Keep tracking by chrony:
            sudo gedit /etc/chrony/chrony.conf
    
    3. comment out the entire pool server
    4. Add Mir's PC as a server at the bottom:
            server 192.168.12.20 minpoll 0 maxpoll 0 maxdelay 0.05

    5. You can verify through these command:
            ntpdate -q 192.168.12.20
            chronyc tracking
            chronyc sources -v
            timedatectl

    6. For ROS setting, set the param "use_sim_time" to True to tell the rospy.time
        to subscribe frome the topic "/clock"

    7. Use this node for publishing "/clock" topic
"""

import rospy
import time

from rosgraph_msgs.msg import Clock

def simtime_talker():
    rospy.set_param('use_sim_time', False)
    time.sleep(1)
    rospy.init_node('sim_clock_node')
    rospy.on_shutdown(set_param_back)
    rospy.set_param('use_sim_time', True)

    pub1 = rospy.Publisher('clock',Clock, queue_size=10)
    rate = rospy.Rate(100) # 10hz
    sim_clock = Clock()

    while(not rospy.is_shutdown()):
        sim_clock.clock = rospy.Time.from_sec(time.time())
        pub1.publish(sim_clock)
        rate.sleep()

def set_param_back():
    rospy.set_param('use_sim_time', False)

if __name__ == '__main__':
    simtime_talker()
    rospy.spin()