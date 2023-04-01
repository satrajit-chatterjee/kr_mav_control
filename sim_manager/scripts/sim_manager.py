#! /usr/bin/env python3
import rospy
import numpy as np
import random

from geometry_msgs.msg import Pose
from nav_msgs.msg import Path, Odometry
from threading import Thread
from std_srvs.srv import Trigger, SetBool
from kr_tracker_msgs.msg import TrajectoryTrackerAction, TrajectoryTrackerGoal, CircleTrackerAction, CircleTrackerGoal

from kr_python_interface.mav_interface import KrMavInterface

waypoints_list = []
present_pos = None

update_waypoints = False

odom_pub = rospy.Publisher("/quadrotor/odom", Odometry, queue_size=10)

odom_published  = False


def plan_callback(path_msg):
    global waypoints_list, update_waypoints, present_pos

    new_waypoints = []
    for pose in path_msg.poses:
        new_waypoints.append(pose)
    
    if len(waypoints_list) == 0 or update_waypoints:
        waypoints_list = new_waypoints
        present_pos = waypoints_list[0]

def plan():
    # replan, subscribe to plan topic and update waypoints list

    # odom publisher is updated by this function

    # if waypoints list is empty, call plan first. 
    # Otherwise, first update odom publisher and then call replan

    pass

def odom_publisher():
    global present_pos, odom_pub
    # Get current position and keep publishing
    odom = Odometry()
    odom.header.frame_id = "odom"
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        odom.header.stamp = rospy.Time.now()
        if present_pos is not None:
            odom.pose.pose = present_pos.pose
            odom_pub.publish(odom)
        rate.sleep()


    # TODO: Have a check here that validates that len(waypoints) always > 0 
    # before publishing in while loop

    pass

def motors_on(mav_name):
    try:
      motors = rospy.ServiceProxy('/' + mav_name + '/mav_services/motors', SetBool)
      resp = motors(True)
      rospy.loginfo(resp)
    except rospy.ServiceException as e:
      rospy.logwarn("Service call failed: %s"%e)
      return 'aborted'


def manager():
    # Create MAV objects
    mav_namespace = 'quadrotor'
    mav_obj = KrMavInterface(mav_namespace, 1)
    # Motor On / Take Off
    mav_obj.motors_on()

    take_off_done = False
    while not take_off_done:
        if present_pos is not None:
            resp = mav_obj.take_off()
            if resp.success:
                take_off_done = True
            else:
                rospy.logwarn("Failed to takeoff, retrying...")
                continue


def main():
    rospy.init_node('sim_manager_node', anonymous=True)

    manager_thread = Thread(target=manager)
    manager_thread.daemon = True
    manager_thread.start()

    # Start odom publisher
    odom_thread = Thread(target=odom_publisher)
    odom_thread.daemon = True
    odom_thread.start()

    # Subscribe to plan
    rospy.Subscriber('/rrt_star_planner/planner/rrt_plan', Path, plan_callback)
    rospy.spin()


    # while not rospy.is_shutdown():
    #     pass

    # rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
