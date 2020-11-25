#!/usr/bin/env python

import sys
import rospy
import rospkg
from time import time, sleep
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped, Twist
from resl_coverage.srv import Neighbors, NeighborsResponse

name = None
myid = None
state = None
status = None
t1 = None
desired = None
kp = 0.75

neighbors_sub = []
neighbors = {}


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

def neighbors_state_callback(msg, arg):
    global neighbors
    p = msg.pose.position
    neighbors[arg] = [p.x, p.y, p.z]


def handle_neighbors_request(req):
    global neighbors_sub, name
    for n in req.neighbors:
        neighbors_sub.append(
                rospy.Subscriber('/unity_command/tracker'+str(n)+'/TrueState/pose',
                    PoseStamped, neighbors_state_callback, n))
    
    res = NeighborsResponse()
    res.rec = True
    return res

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

def check_no_collision(state):
    global neighbors, myid

    for k, v in neighbors.items():
        if k < myid:
            continue

        dx = state[0] - v[0]
        dy = state[1] - v[1]
        dz = state[2] - v[2]
        if (dx**2 + dy**2 + dz**2) < 4:
            return False 
    return True

def main():
    global state, name, t1, desired, myid
    myid = int(name[1:-1].replace('tracker', ''))
    t1 = time()
    desired = [0, 2, 5, 0, 0, 0]
    desired[0] = float(sys.argv[1])
    desired[1] = float(sys.argv[2])
    desired[2] = float(sys.argv[3])

    neighbors_res = rospy.Service(name+'neighbors', Neighbors, handle_neighbors_request)

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
            myid = int(name[1:-1].replace('tracker', ''))
            #print(myid, vel)
            alts = []
            for i in range(3):
                alts.append( 1.5 * vel[i] / abs(vel[i]) if vel[i] else 0.)
            for i in range(3, 6):
                alts.append( 0.2 * vel[i] / abs(vel[i]) if vel[i] else 0.)

            if check_no_collision(state):
                twist.linear.x = min(vel[0], alts[0], key=abs)
                twist.linear.y = min(vel[1], alts[1], key=abs)
                twist.linear.z = min(vel[2], alts[2], key=abs)
            else:
                twist.linear.x = 0.
                twist.linear.y = 0.
                twist.linear.z = 0.
            twist.angular.x = min(vel[3], alts[3], key=abs)
            twist.angular.y = min(vel[4], alts[4], key=abs)
            twist.angular.z = min(vel[5], alts[5], key=abs)
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
