#!/usr/bin/env python

import sys
import rospy
import matplotlib.pyplot as plt
from time import time
from geometry_msgs.msg import PoseStamped, TwistStamped
from resl_coverage.msg import StateEstimate
from math import sqrt

num_targets = int(sys.argv[1])
num_trackers = int(sys.argv[2])

truth = [None, None, None, None]
estimate = [None, None, None, None]
update = [False, False, False]


def true_pose_callback(msg):
    global truth, update
    truth[0] = msg.pose.position.x
    truth[1] = msg.pose.position.y
    update[0] = True


def true_twist_callback(msg):
    global truth, update
    truth[2] = msg.twist.linear.x
    truth[3] = msg.twist.linear.y
    update[1] = True


def estimate_callback(msg):
    global estimate, update
    if msg.id == 0:
        estimate = [msg.pose.position.x, msg.pose.position.y,
                    msg.twist.linear.x, msg.twist.linear.y] 
        update[2] = True


def record():
    rospy.init_node('resl_recorder')

    # Initialize Subscribers
    pose_sub = rospy.Subscriber('/unity_command/target0/TrueState/pose', PoseStamped, true_pose_callback)
    twist_sub = rospy.Subscriber('/unity_command/target0/TrueState/twist', TwistStamped, true_twist_callback)
    estimate_sub = rospy.Subscriber('/target_estimates', StateEstimate, estimate_callback)

    # plt.ylim(ymin=0, ymax=1) # If MSE is large we will get a blank plot

    plt.xlabel('Time')
    plt.ylabel('Squared Error')
    plt.title('Estimation Error')
    rx = []
    ry = []
    f = open('logs/states.log', 'w')
    global truth, estimate, update
    t0 = time()
    rate = rospy.Rate(1)
    counter = 0
    while not rospy.is_shutdown():
        if all(update):
            epx = abs(truth[0] - estimate[0])
            epy = abs(truth[1] - estimate[1])
            evx = abs(truth[2] - estimate[2])
            evy = abs(truth[3] - estimate[3])
            mse = sqrt((epx**2 + epy**2 + evx**2 + evy**2))
            ti = time()
            
            print("truth_x:{}, est_x:{}".format(truth[0],estimate[0]))
            print("truth_y:{}, est_y:{}".format(truth[1],estimate[1]))

            rx.append(ti - t0)
            ry.append(mse)
            plt.plot(rx, ry, color='blue')
            plt.pause(0.05)

            f.write("%d,%s,t\n" % (counter, ",".join(map(str, truth))))
            f.write("%d,%s,e\n" % (counter, ",".join(map(str, estimate))))

            update = [False, False, False]
            counter += 1

        rate.sleep()

    plt.show()
    f.close()


if __name__ == "__main__":
    try:
        record()
    except rospy.ROSInterruptException:
        pass
