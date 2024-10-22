#!/usr/bin/env python3


import rospy
import time
import argparse

import pandas as pd
import matplotlib.pyplot as plt



if __name__ == '__main__':

    df = pd.read_csv('test.csv',index_col="time")
    fig, axes = plt.subplots(nrows=2, ncols=1)

    df["target_x"].plot(ax=axes[0])
    df["center_x"].plot(ax=axes[0])
    df["target_y"].plot(ax=axes[1])
    df["center_y"].plot(ax=axes[1])

    plt.show() 

    """
    parser = argparse.ArgumentParser(description='ROS to csv for experimenting')
    parser.add_argument('-n', type=str, help='record file name', default="test")
    parser.add_argument('-t', type=float, help='period time(second)', default=0.1)

    args = parser.parse_args()
    PIDRecorder(file_name=args.n, period=args.t)
    """