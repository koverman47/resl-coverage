#!/usr/bin/env python

import sys
import rospy
import numpy as np
from time import time
from kalman import Kalman
from numpy.linalg import pinv
from geometry_msgs.msg import PoseStamped, TwistStamped


known = False
num_targets = int(sys.argv[1])
num_trackers = int(sys.argv[2])

# Declare Publishers
desired_pose = None
des_pub = None

# Declare Subscribers
target_pose_subs = []
target_twist_subs = []

z = np.array([None, None, None, None, None, None])
P = np.dot(0.01, np.eye(6))
x = np.array([0., 0., 0., 0., 0., 0.])
offset = [float(sys.argv[3]), float(sys.argv[4]), 0.]

A = None
B = None
H = None
u = None
Q = None
R = None

set_est = [False, False]
def pose_callback(msg):
    global z, x, set_est, R
    noise = np.dot(R, np.random.random(6))
    z[0] = msg.pose.position.x + (noise[0] * 0.01)
    z[1] = msg.pose.position.y + (noise[1] * 0.01)
    z[2] = msg.pose.position.z + (noise[2] * 0.01)
    if not set_est[0]:
        x[0] = msg.pose.position.x
        x[1] = msg.pose.position.y
        x[2] = msg.pose.position.z
        set_est[0] = True

def twist_callback(msg):
    global z, x, set_est, R, u
    noise = np.dot(R, np.random.random(6))
    z[3] = msg.twist.linear.x + (noise[0] * 0.01)
    z[4] = msg.twist.linear.y + (noise[1] * 0.01)
    z[5] = msg.twist.linear.z + (noise[2] * 0.01)

    u[0] = msg.twist.linear.x
    u[1] = msg.twist.linear.y
    u[2] = msg.twist.linear.z
    if not set_est[1]:
        x[3] = msg.twist.linear.x
        x[4] = msg.twist.linear.y
        x[5] = msg.twist.linear.z
        set_est[1] = True

def init_messages():
    global name
    global num_targets, num_trackers

    # Publishers
    global desired_pose, des_pub
    desired_pose = PoseStamped()
    des_pub = rospy.Publisher(name+'desired_pose', PoseStamped, queue_size=1)


    # Subscription
    global target_twist_subs, target_pose_subs
    for i in range(num_targets):
        target_pose_subs.append(
                rospy.Subscriber('/unity_command/target'+str(i)+'/TrueState/pose', 
                    PoseStamped, pose_callback))
        target_twist_subs.append(
                rospy.Subscriber('/unity_command/target'+str(i)+'/TrueState/twist',
                    TwistStamped, twist_callback))


def init_params():
    global name
    global num_targets
    global A, B, u, Q, H, R

    name = rospy.get_namespace()

    A = np.eye(6)
    B = np.concatenate((np.zeros((3, 3)), np.eye(3)))
    u = np.ones(3)
    Q = np.eye(6)
    #Q[0][3] = 1.
    #Q[1][4] = 1.
    #Q[2][5] = 1.
    #Q[3][0] = 1.
    #Q[4][1] = 1.
    #Q[5][2] = 1.
    H = np.eye(6)
    R = np.eye(6)


def track():
    global num_targets, num_trackers
    global z, x, P, offset
    global A, B, u, Q, H, R
    global N, edges, node_weights
    global desired_pose, des_pub
    global information_pub, state_information

    rospy.init_node(name[1:-1] + '_tracking')

    desired_pose.pose.position.x = offset[0]
    desired_pose.pose.position.y = offset[1]
    desired_pose.pose.position.z = offset[2]
    
    # Init time
    t1 = time()
    dt = 0.

    rospy.sleep(2)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        t2 = time()
        dt = t2 - t1
        t1 = t2
        
        A[0][3] = dt / 2.
        A[1][4] = dt / 2.
        A[2][5] = dt / 2.
        B[0][0] = dt / 2.
        B[1][1] = dt / 2.
        B[2][2] = dt / 2.
        I = np.eye(6)

        global known
        if known:
            pred_x = np.dot(A, x) + np.dot(B, u)
            pred_P = np.dot(A, np.dot(P, A.T)) + Q
            K = np.dot(pred_P, np.dot(H.T, pinv(np.dot(H, np.dot(pred_P, H.T)) + R)))
            x = pred_x + np.dot(K, z - np.dot(H, pred_x))
            P = np.dot(np.eye(6) - np.dot(K, H), pred_P)
        else:
            pred_x = np.dot(A, x)
            pred_P = np.dot(A, np.dot(P, A.T)) + Q
            RT = R + np.dot(H, np.dot(pred_P, H.T))
            F = np.dot(H, B)
            M = np.dot(pinv(np.dot(F.T, np.dot(pinv(RT), F))), np.dot(F.T, pinv(RT)))
            u = np.dot(M, z - np.dot(H, pred_x))
            pred_x_star = pred_x + np.dot(B, u)
            IBMH = I - np.dot(B, np.dot(M, H))
            pred_P_star = np.dot(IBMH, np.dot(pred_P, IBMH.T))
            K = np.dot(pred_P, np.dot(H.T, pinv(RT)))
            x = pred_x_star + np.dot(K, z - np.dot(H, pred_x_star))
            P = pred_P_star - np.dot(K, np.dot(H, pred_P_star) - np.dot(R, np.dot(M.T, B.T)))

        for i in range(len(P)):
            for j in range(len(P[i])):
                if P[i][j] < 0.:
                    P[i][j] = 0.
            
        print("x")
        print(x)
        print("P")
        print(P)

        desired_pose.pose.position.x = x[0] + offset[0]
        desired_pose.pose.position.y = x[1] + offset[1]
        desired_pose.pose.position.z = max(x[2], 3.) + offset[2]
        des_pub.publish(desired_pose)

        rate.sleep()


if __name__ == "__main__":
    try:
        init_params()
        init_messages()
        track()
    except rospy.ROSInterruptException:
        pass
