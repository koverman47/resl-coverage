#!/usr/bin/env python

import sys
import rospy
import time
from math import sqrt
from geometry_msgs.msg import PoseStamped


pose = [0., 0., 0.]
tolerance = 0.5
trackers_started = False

def pose_callback(msg):
    global pose
    pose[0] = msg.pose.position.x
    pose[1] = msg.pose.position.y
    pose[2] = msg.pose.position.z

def tracker_callback(msg):
    global trackers_started
    trackers_started = True

'''
' des :- PoseStamped
' cur :- list
'''
def distance(des, cur):
    x = des.pose.position.x - cur[0]
    y = des.pose.position.y - cur[1]
    z = des.pose.position.z - cur[2]
    sqer = sqrt(x**2 + y**2 + z**2)
    return sqer

def mission():
    global pose, tolerance, trackers_started
    name = rospy.get_namespace()
    rospy.init_node(name[1:-1] + '_mission')

    pose_sub = rospy.Subscriber('/unity_command'+name+'TrueState/pose', PoseStamped, pose_callback)
    des_pub = rospy.Publisher(name+'desired_pose', PoseStamped, queue_size=1)
    des_sub = rospy.Subscriber('/tracker0/desired_pose', PoseStamped, tracker_callback)
    desired_pose = PoseStamped()

    in_x = float(sys.argv[1])
    in_y = float(sys.argv[2])
    in_z = float(sys.argv[3])
    desired_pose.pose.position.x = in_x
    desired_pose.pose.position.y = in_y
    desired_pose.pose.position.z = in_z

    waypoints = []
    waypoints.append([0., 0., 5.])
    waypoints.append([5., 2., 8.])
    waypoints.append([5., 2., 5.])
    waypoints.append([2., 2., 5.])
    waypoints.append([2., 0., 5.])
    
    while not trackers_started:
        pass
    des_sub.unregister()

    print("started target")
    stability = 0
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if distance(desired_pose, pose) < tolerance:
            stability += 1
        else:
            stability = 0
        if distance(desired_pose, pose) < tolerance and waypoints and stability > 50:
            wp = waypoints.pop()
            desired_pose.pose.position.x = wp[0] + in_x
            desired_pose.pose.position.y = wp[1] + in_y
            desired_pose.pose.position.z = max(wp[2], 3.)
            print("New Waypoint", wp)
        des_pub.publish(desired_pose)
        rate.sleep()

if __name__ == "__main__":
    try:
        mission()
    except rospy.ROSInterruptException:
        pass
