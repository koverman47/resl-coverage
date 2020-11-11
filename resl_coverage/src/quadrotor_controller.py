#!/usr/bin/env python

import sys
import rospy
import rospkg
from geometry_msgs.msg import PoseStamped, Twist
from tf.transformations import euler_from_quaternion
from time import time, sleep

name = None
state = None
status = None
t1 = None
desired = None
kp = 0.75

'''
' Update State from unity pose publisher
' msg : PoseStamped
'''
def state_callback(msg):
    global state
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z
    quat = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    roll, pitch, yaw = euler_from_quaternion(quat)
    state = [x, y, z, roll, pitch, yaw]

'''
' Update desired pose
' msg : PoseStamped
'''
def desired_callback(msg):
    global desired
    desired[0] = msg.pose.position.x
    desired[1] = msg.pose.position.y
    desired[2] = max(msg.pose.position.z, 3.)

def get_velocity(d, x):
    global kp
    v = []
    v.append(-kp * (x[0] - d[0]))
    v.append(-kp * (x[1] - d[1]))
    v.append(-kp * (x[2] - d[2]))
    v.append(-kp * (x[3] - d[3]))
    v.append(-kp * (x[4] - d[4]))
    v.append(-kp * (x[5] - d[5]))
    return v

def main():
    global state, name, t1, desired
    t1 = time()
    desired = [0, 2, 5, 0, 0, 0]
    desired[0] = float(sys.argv[1])
    desired[1] = float(sys.argv[2])
    desired[2] = float(sys.argv[3])

    sub_pose = rospy.Subscriber('/unity_command'+name+'TrueState/pose', PoseStamped, state_callback)
    sub_des = rospy.Subscriber(name+'desired_pose', PoseStamped, desired_callback)
    twist = Twist()
    pub_twist = rospy.Publisher(name+'cmd_vel', Twist, queue_size=1)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if state:
            t2 = time()
            dt = t2 - t1
            t1 = t2
            vel = get_velocity(desired, state)
            twist.linear.x = min(vel[0], 2.)
            twist.linear.y = min(vel[1], 2.)
            twist.linear.z = min(vel[2], 2.)
            twist.angular.x = min(vel[3], 0.25)
            twist.angular.y = min(vel[4], 0.25)
            twist.angular.z = min(vel[5], 0.25)
            pub_twist.publish(twist)
        rate.sleep()

def initialize():
    global name
    name = rospy.get_namespace()
    rospy.init_node(name[1:-1] + '_controller')

if __name__ == "__main__":
    try:
        initialize()
        main()
    except rospy.ROSInterruptException:
        pass
