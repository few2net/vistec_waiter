#!/usr/bin/env python3


import rospy
import time
import argparse
import pandas as pd

from geometry_msgs.msg import Vector3
from rosgraph_msgs.msg import Clock


class PIDRecorder:
    def __init__(self, file_name="test", period=0.1):
        rospy.init_node('pid_recorder')
        rospy.on_shutdown(self.save_to_csv)

        self.file_name = file_name
        self.period = period
        self.target_temp = [0,0]
        self.center_temp = [0,0]
        self.df = pd.DataFrame(columns=["time",
                                        "target_x",
                                        "target_y",
                                        "center_x",
                                        "center_y"])


        rospy.Subscriber('/test_pid/target', Vector3, self.target_rec)
        rospy.Subscriber('/test_pid/current', Vector3, self.center_rec)

        # Run
        print("Start Recording...")
        self.stamp = 0.0
        rospy.Timer(rospy.Duration(self.period), self.period_record)
        rospy.spin()

    def target_rec(self, data):
        self.target_temp[0] = data.x
        self.target_temp[1] = data.y

    def center_rec(self, data):
        self.center_temp[0] = data.x
        self.center_temp[1] = data.y

    def period_record(self, event):
        data = {"time": round(self.stamp, 4),
                "target_x": self.target_temp[0],
                "target_y": self.target_temp[1],
                "center_x": self.center_temp[0],
                "center_y": self.center_temp[1]}
        self.df = self.df.append(data, ignore_index=True)
        self.stamp += self.period

    def save_to_csv(self):
        self.df.to_csv(self.file_name + '.csv', index=False)
        print(f"Saving to {self.file_name}.csv .....")
        time.sleep(2)
        print("Finish")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='ROS to csv for experimenting')
    parser.add_argument('-n', type=str, help='record file name', default="test")
    parser.add_argument('-t', type=float, help='period time(second)', default=0.1)

    args = parser.parse_args()
    PIDRecorder(file_name=args.n, period=args.t)