#! /usr/bin/env python3
import rospy
import numpy as np
import random

from geometry_msgs.msg import Pose
from nav_msgs.msg import Path
from kr_tracker_msgs.msg import TrajectoryTrackerAction, TrajectoryTrackerGoal, CircleTrackerAction, CircleTrackerGoal

from kr_python_interface.mav_interface import KrMavInterface



def plan():
    # replan, subscribe to plan topic and update waypoints list

    # odom publisher is updated by this function

    # if waypoints list is empty, call plan first. 
    # Otherwise, first update odom publisher and then call replan

    pass

def odom_publisher():
    # get current position and keep publishing
    pass

def main():
    rospy.init_node('sim_manager_node', anonymous=True)

    # Create MAV objects
    mav_namespace = 'quadrotor'
    mav_id = 1
    mav_obj = KrMavInterface(mav_namespace,  mav_id)

    # Motor On / Take Off
    mav_obj.motors_on()
    mav_obj.take_off()

    while not rospy.is_shutdown():
        pass

    # rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
