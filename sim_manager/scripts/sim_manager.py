#! /usr/bin/env python2.7
import rospy
import numpy as np
import random

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from threading import Thread
from std_srvs.srv import SetBool
import dubins
import tf
from visualization_msgs.msg import Marker, MarkerArray
from kr_tracker_msgs.msg import TrajectoryTrackerAction, TrajectoryTrackerGoal, CircleTrackerAction, CircleTrackerGoal
from geometry_msgs.msg import Point, Quaternion, Vector3
from kr_python_interface.mav_interface import KrMavInterface
from ompl_rrtstar.srv import ReplanTrigger, ReplanTriggerRequest

waypoints_list = []
present_pos = None

frame_id = "odom"

update_waypoints = False

odom_pub = rospy.Publisher("/quadrotor/odom", Odometry, queue_size=10)
odom_published  = False

fn_waypt_pub = rospy.Publisher("/sim_manager/final_waypoints", Marker, queue_size=100)

robot_marker_pub = rospy.Publisher('robot_marker', Marker, queue_size=1)

min_turn_radius = 0.01
path_resolution = 10

replan_service = '/rrt_star_planner/replan'


def convert_2_euler(q):
    euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
    roll, pitch, yaw = euler
    return [roll, pitch, yaw]


def plan_callback(path_msg):
    global waypoints_list, update_waypoints, present_pos, min_turn_radius, path_resolution, frame_id

    new_waypoints = []
    for pose in path_msg.poses:
        new_waypoints.append(pose)
    
    # create intermediate waypoints
    final_waypoints = []
    for wp_idx in range(len(new_waypoints)-1):
        start = np.array([new_waypoints[wp_idx].pose.position.x, 
                    new_waypoints[wp_idx].pose.position.y,
                    new_waypoints[wp_idx].pose.position.z]  )

        goal = np.array([new_waypoints[wp_idx + 1].pose.position.x, 
                    new_waypoints[wp_idx + 1].pose.position.y,
                    new_waypoints[wp_idx + 1].pose.position.z]  )

        n = 10

        # Compute the vector between p1 and p2
        line_vector = goal - start
        # Generate n evenly spaced points along the line
        points = np.linspace(0, 1, path_resolution)[:, np.newaxis] * line_vector[np.newaxis, :] + start[np.newaxis, :]

        for p in range(1, len(points)-1):
            # Convert the sampled points into waypoints 
            waypt_pose = PoseStamped()
            waypt_pose.header.frame_id = frame_id
            waypt_pose.header.stamp = rospy.Time.now()

            waypt_pose.pose.position.x = points[p][0]
            waypt_pose.pose.position.y = points[p][1]
            waypt_pose.pose.position.z = points[p][2]

            waypt_pose.pose.orientation = new_waypoints[wp_idx].pose.orientation
            final_waypoints.append(waypt_pose)

    # Append final waypoint into final waypoints list
    final_waypoints.append(new_waypoints[-1])

    if len(waypoints_list) == 0 or update_waypoints:
        waypoints_list = final_waypoints
        present_pos = waypoints_list[0]


def final_waypt_publisher():
    global waypoints_list, frame_id, fn_waypt_pub
    
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "pose_array"
    marker.id = 1
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.1
    marker.color.b = 1.0
    marker.color.a = 1.0
    
    for i, pose in enumerate(waypoints_list):
        point = Point()
        point.x = pose.pose.position.x
        point.y = pose.pose.position.y
        point.z = pose.pose.position.z
        marker.points.append(point)
    fn_waypt_pub.publish(marker)


def robot_pose_publisher():
    global present_pos
     # Create a marker message for robot
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "robot"
    marker.id = 3;
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose = present_pos.pose
    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = 0.5

    marker.color.r = 1.0
    marker.color.a = 1.0

    final_waypt_publisher()    
    # Publish the marker
    robot_marker_pub.publish(marker)



def odom_publisher():
    global present_pos, odom_pub, robot_marker_pub
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


def manager():
    global waypoints_list, present_pos

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
    
    while not rospy.is_shutdown():
        # Follow waypoints
        waypoint_idx = 0
        while waypoint_idx < len(waypoints_list):
            rospy.logwarn("Going to waypoint # " + str(waypoint_idx) + " of " + str(len(waypoints_list)))
            waypoint = waypoints_list[waypoint_idx]
            resp = mav_obj.send_wp(waypoint.pose.position.x,
                            waypoint.pose.position.y,
                            waypoint.pose.position.z,
                            convert_2_euler(waypoint.pose.orientation)[-1])
            waypt_success = False
            while not waypt_success:
                waypt_success = resp.success
                rospy.sleep(10)
            rospy.logwarn("Waypoint " + str(waypoint_idx) + " reached successfully")
            # Update odom
            present_pos = waypoint
            robot_pose_publisher()

            waypoint_idx += 1
        
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


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
