#!/usr/bin/env python

import sys
import rospy
import rospkg
from math import sqrt, cos, sin
from geometry_msgs.msg import PoseStamped, Twist
from resl_coverage.srv import Neighbors, NeighborsResponse
from tf.transformations import euler_from_quaternion
from time import time, sleep

name = None
myid = None
state = None
status = None
t1 = None
desired = None
kp = 1.0

# Declare Message Info
sub_pose = None
sub_des = None
twist = None
pub_twist = None

neigbors_sub = []
neighbors = {}


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

def neighbor_state_callback(msg, arg):
    global neighbors
    p = msg.pose.position
    neighbors[arg] = [p.x, p.y]

def handle_neighbors_request(req):
    global neighbors_sub, name
    for n in req.neighbors:
        neighbors_sub.append(
                rospy.Subscriber('/unity_command/target'+str(n)+'/TrueState/pose',
                    PoseStamped, neighbors_state_callback(n)))
    res = NeighborsResponse()
    res.rec = True
    return res


def get_velocity(d, x):
    global kp
    v = []
    v.append(-kp * (x[0] - d[0]))
    v.append(-kp * (x[1] - d[1]))
    v.append(-kp * (x[2] - d[2]))
    
    return v

def distance(d, s):
    x = s[0] - d[0]
    y = s[1] - d[1]
    return sqrt(x**2 + y**2)

def check_no_collision(state):
    global neighbors, myid

    for k, v in neighbors.items():
        if k < myid:
            continue

        dx = state[0] - v[0]
        dy = state[1] - v[1]
        if (dx**2 + dy**2) < 4:
            return False
    return True


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
    global twist, pub_twist, myid
    myid = int(name[1:-1].replace('target', ''))
    #neighbors_res = rospy.Service(name+'neighbors', Neighbors, handle_neighbors_request)
    if myid == 0:
        sub = rospy.Subscriber('/unity_command/target1/TrueState/pose',
                PoseStamped, neighbor_state_callback, 1)
    else:
        sub = rospy.Subscriber('/unity_command/target0/TrueState/pose',
                PoseStamped, neighbor_state_callback, 0)
    t1 = time()
    desired = [0, 0, 0]
    desired[0] = float(sys.argv[1])
    desired[1] = float(sys.argv[2])
    desired[2] = float(sys.argv[3])

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if state:
            dist = distance(desired, state)
            if dist == 0.:
                dira = 0.2
            else:
                dira = cos((desired[0] - state[0]) / dist)
            if False: #dist > 0.2 and abs(dira - state[2]) > 0.1:
                vel = get_velocity([0., 0., dira], state)
                twist.linear.x = 0.
                twist.linear.y = 0.
                twist.angular.z = vel[2]
            elif True: #dist > 0.2:
                if check_no_collision(state):
                    vel = get_velocity(desired, state)
                    alt = []
                    alt.append(1. * vel[0] / abs(vel[0]) if vel[0] else 0.)
                    alt.append(1. * vel[1] / abs(vel[1]) if vel[1] else 0.)
                    alt.append(0.5 * vel[2] / abs(vel[2]) if vel[2] else 0.)
                    twist.linear.x = min(vel[0], alt[0], key=abs)
                    twist.linear.y = min(vel[1], alt[1], key=abs)
                    twist.angular.z = min(vel[2], alt[2], key=abs)
                else:
                    twist.linear.x = 0.
                    twist.linear.y = 0.
                    twist.angular.z = 0.
            elif dist < 0.2 and abs(desired[2] - state[2]) > 0.1:
                vel = gel_velocity(desired, state)
                twist.linear.x = 0.
                twist.linear.y = 0.
                twist.angular.z = vel[2]
            #print(twist.linear.x, twist.linear.y, twist.angular.z)
            pub_twist.publish(twist)
        else:
            print("no state")
        rate.sleep()


if __name__ == "__main__":
    try:
        initialize()
        main()
    except rospy.ROSInterruptException:
        pass
