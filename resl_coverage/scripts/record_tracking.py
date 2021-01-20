#!/usr/bin/env python

import sys
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from resl_coverage.msg import StateEstimate

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
        estimate = msg.state
        update[2] = True


def record():
    rospy.init_node('resl_recorder')

    # Initialize Subscribers
    pose_sub = rospy.Subscriber('/unity_command/target0/TrueState/pose',
            PoseStamped, true_pose_callback)
    twist_sub = rospy.Subscriber('/unity_command/target0/TrueState/twist',
            TwistStamped, true_twist_callback)

    estimate_sub = rospy.Subscriber('/target_estimates',
            StateEstimate, estimate_callback)


    f = open('logs/states.log', 'w')
    global truth, estimate, update
    rate = rospy.Rate(1)
    counter = 0
    while not rospy.is_shutdown():
        if all(update):
            f.write("%d,%s,t\n" % (counter, ",".join(map(str, truth))))
            f.write("%d,%s,e\n" % (counter, ",".join(map(str, estimate)))) 

            update = [False, False, False]
            counter += 1

        rate.sleep()

    f.close()


if __name__ == "__main__":
    try:
        record()
    except rospy.ROSInterruptException:
        pass
