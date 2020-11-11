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

# Declare Message Info
sub_pose = None
sub_des = None
twist = None
pub_twist = None


def state_callback(msg):
    global state
    x = msg.pose.position.x
    y = msg.pose.position.y
    a = msg.pose.orientation
    quat = [a.x, a.y, a.z, a.w]
    roll, pitch, yaw = euler_from_quaternion(quat)
    state = [x, y, yaw]


def desired_callback(msg):
    global desired
    desired[0] = msg.pose.position.x
    desired[1] = msg.pose.position.y
    desired[2] = msg.pose.orientation.z


def get_velocity(d, x):
    global kp
    v = []
    v.append(-kp * (x[0] - d[0]))
    v.append(-kp * (x[1] - d[1]))
    v.append(-kp * (x[2] - d[2]))
    
    return v


def initialize():
    global name
    global sub_pose, sub_des
    global pub_twist, twist
    name = rospy.get_namespace()
    rospy.init_node(name[1:-1] + '_controller')

    sub_pose = rospy.Subscriber('/unity_command'+name+'TrueState/pose',
            PoseStamped, state_callback)
    sub_des = rospy.Subscriber(name+'desired_pose', PoseStamped, desired_callback)

    twist = Twist()
    pub_twist = rospy.Publisher(name+'cmd_vel', Twist, queue_size=1)


def main():
    global state, name, t1, desired
    global twist, pub_twist
    t1 = time()
    desired = [0, 0, 0]
    desired[0] = float(sys.argv[1])
    desired[1] = float(sys.argv[2])
    desired[2] = float(sys.argv[3])

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if state:
            t2 = time()
            dt = t2 - t1
            t1 = t2
            vel = get_velocity(desired, state)
            twist.linear.x = min(vel[0], 1.)
            twist.linear.y = min(vel[1], 1.)
            twist.angular.z = min(vel[2], 0.25)
            pub_twist.publish(twist)
        rate.sleep()


if __name__ == "__main__":
    try:
        initialize()
        main()
    except rospy.ROSInterruptException:
        pass
