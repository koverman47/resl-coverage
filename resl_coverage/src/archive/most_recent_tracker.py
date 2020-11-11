#!/usr/bin/env python
import sys
import rospy
import numpy as np
import networkx as nx
from time import time
from kalman import Kalman
from numpy.linalg import pinv
from geometry_msgs.msg import PoseStamped, TwistStamped
from resl_resilient_tracking.msg import MultiStateEstimate
from resl_resilient_tracking.srv import Topology, TopologyResponse
from resl_resilient_tracking.srv import ProcessNoise, ProcessNoiseResponse
from resl_resilient_tracking.srv import Failure, FailureResponse
from resl_resilient_tracking.srv import Coordinates, CoordinatesResponse
from resl_resilient_tracking.srv import TriggerFail, TriggerFailResponse

num_targets = int(sys.argv[1])
num_trackers = int(sys.argv[2])

# Declare Services
process_noise_res = None 
failure_res = None
des_offset_res = None
topology_res = None
trigger_fail_res = None

# Declare Message Objects
state_information = None
desired_pose = None

# Declare Publishers
des_pub = None
information_pub = None

# Declare Subscribers
offset_sub = None
target_pose_subs = []
target_twist_subs = []
consensus_subs = []

myid = None
#irec = [False for i in range(num_trackers)]
irec = {}
information_q = {} # Update to dictionary
information_W = {} # Update to dictionary
N = 1
edges = []
weight_matrix = None

obs = [[None, None, None, None, None, None] for i in range(num_targets)]
#covariances = [np.diag((0.01, 0.01, 0.01, 0.01, 0.01, 0.01)) for i in range(num_targets)]
covariances = [np.dot(0.01, np.eye(6)) for i in range(num_targets)]
estimates = [np.array([0., 0., 0., 0., 0., 0.]) for i in range(num_targets)]
offset = [float(sys.argv[3]), float(sys.argv[4]), 0.]

A = None
B = None
H = None
U = None
Q = None
R = None

set_est = [[False, False] for i in range(num_targets)]
def pose_callback(msg, args):
    global obs, estimates, set_est, R
    noise = np.dot(R, np.random.random(6))
    obs[args][0] = msg.pose.position.x + (noise[0] * 0.01)
    obs[args][1] = msg.pose.position.y + (noise[1] * 0.01)
    obs[args][2] = msg.pose.position.z + (noise[2] * 0.01)
    if not set_est[args][0]:
        estimates[args][0] = msg.pose.position.x
        estimates[args][1] = msg.pose.position.y
        estimates[args][2] = msg.pose.position.z
        set_est[args][0] = True

def twist_callback(msg, args):
    global obs, estimates, set_est, R, U
    noise = np.dot(R, np.random.random(6))
    obs[args][3] = msg.twist.linear.x + (noise[0] * 0.01)
    obs[args][4] = msg.twist.linear.y + (noise[1] * 0.01)
    obs[args][5] = msg.twist.linear.z + (noise[2] * 0.01)

    U[args][0] = msg.twist.linear.x
    U[args][1] = msg.twist.linear.y
    U[args][2] = msg.twist.linear.z
    if not set_est[args][1]:
        estimates[args][3] = msg.twist.linear.x
        estimates[args][4] = msg.twist.linear.y
        estimates[args][5] = msg.twist.linear.z
        set_est[args][1] = True

def offset_callback(msg):
    global offset
    offset[0] = msg.pose.position.x
    offset[1] = msg.pose.position.y
    offset[2] = msg.pose.position.z

def information_callback(msg):
    global irec, information_q, information_W
    global num_targets
    information_q[msg.id] = np.array(msg.q).reshape((num_targets, 6))
    information_W[msg.id] = np.array(msg.W).reshape((num_targets, 6, 6))
    irec[msg.id] = True

def handle_topology(req):
    global edges, weight_matrix, N
    global num_trackers
    global irec, myid

    edges = req.edges
    for e in edges:
        irec[e] = False

    N = len(edges)
    weight_matrix = np.array(req.weight_matrix).reshape((num_trackers, num_trackers))

    global consensus_subs
    for j in range(num_trackers):
        if j != myid and j in edges:
            consensus_subs.append(
                    rospy.Subscriber('/tracker'+str(j)+'/state_information',
                        MultiStateEstimate, information_callback))

    res = TopologyResponse()
    res.rec = 1
    return res

def handle_offsets(req):
    global offset
    offset[0] = req.x[0]
    offset[1] = req.y[1]
    offset[2] = req.z[2]

    res = CoordinatesResponse()
    res.rec = 1
    return res

def handle_process_noise(req):
    global R
    return ProcessNoiseResponse(R.flatten())

def handle_failure(req):
    global estimates, covariances
    global consensus_subs, edges
    global num_targets
    edges = []
    consensus_subs = []
    irec = {}


    res = FailureResponse()
    #inter_W = np.array([pinv(covariances[i]) for i in range(num_targets)])
    #res.W = inter_W.flatten()
    res.W = np.array([np.eye(6) for i in range(num_targets)]).flatten()
    res.x = np.array(estimates).flatten()
    #res.P = np.array(covariances).flatten()
    res.P = np.array([np.eye(6) for i in range(num_targets)]).flatten()
    return res

def handle_trigger_fail(req):
    global R
    R = np.dot(1.1, R)
    res = TriggerFailResponse()
    return res

def init_services():
    global name
    global process_noise_res, failure_res
    global des_offset_res, topology_res
    global trigger_fail_res

    topology_res = rospy.Service(name+'topology', Topology, handle_topology)
    des_offset_res = rospy.Service(name+'desired_offsets', Coordinates, handle_offsets)
    process_noise_res = rospy.Service(name+'process_noise', ProcessNoise, handle_process_noise)
    failure_res = rospy.Service(name+'failure', Failure, handle_failure)
    trigger_fail_res = rospy.Service(name+'trigger_fail', TriggerFail, handle_trigger_fail)

def init_messages():
    global name
    global num_targets, num_trackers

    # Publication
    global state_information, desired_pose
    global des_pub, information_pub
    state_information = MultiStateEstimate()
    desired_pose = PoseStamped()

    des_pub = rospy.Publisher(name+'desired_pose', PoseStamped, queue_size=1)
    information_pub = rospy.Publisher(name+'state_information', 
                        MultiStateEstimate, queue_size=2*num_trackers)

    # Subscription
    global offset_sub, target_pose_subs
    global target_twist_subs
    offset_sub = rospy.Subscriber(name+'offset', PoseStamped, offset_callback)
    for i in range(num_targets):
        target_pose_subs.append(
                rospy.Subscriber('/unity_command/target'+str(i)+'/TrueState/pose', 
                    PoseStamped, pose_callback, i))
        target_twist_subs.append(
                rospy.Subscriber('/unity_command/target'+str(i)+'/TrueState/twist',
                    TwistStamped, twist_callback, i))


def init_params():
    global name, myid
    global num_targets
    global A, B, U, Q, H, R

    name = rospy.get_namespace()
    myid = int(name[1:-1].replace('tracker', ''))

    A = np.eye(6)
    B = np.concatenate((np.eye(3), np.eye(3)))
    U = [np.ones(3) for i in range(num_targets)]
    Q = np.eye(6)
    #Q[0][3] = 1.
    #Q[1][4] = 1.
    #Q[2][5] = 1.
    #Q[3][0] = 1.
    #Q[4][1] = 1.
    #Q[5][2] = 1.
    H = np.eye(6)
    R = np.eye(6)

    #global covariances
    #val = 0.5
    #for i in range(num_targets):
    #    covariances[i][0][3] = val
    #    covariances[i][1][4] = val
    #    covariances[i][2][5] = val
    #    covariances[i][3][0] = val
    #    covariances[i][4][1] = val
    #    covariances[i][5][2] = val
    pass


def track():
    global num_targets, num_trackers, myid
    global irec, q, W, information_q, information_W
    global obs, offset, estimates, covariances
    global A, B, U, Q, H, R
    global N, edges, node_weights
    global desired_pose, des_pub
    global information_pub, state_information

    rospy.init_node(name[1:-1] + '_tracking')

    desired_pose.pose.position.x = offset[0]
    desired_pose.pose.position.y = offset[1]
    desired_pose.pose.position.z = offset[2]
    
    # Init Kalman Object
    kalman = Kalman(A, B, H, Q, R)

    # Init time
    t1 = time()
    dt = 0.

    rospy.sleep(2)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        while not edges:
            pass

        t2 = time()
        dt = t2 - t1
        t1 = t2
        
        # update to center on group of targets
        q = []
        W = []
        if all([ob[0] for ob in obs]):

            for i in range(num_targets):
                A[0][3] = dt / 2.
                A[1][4] = dt / 2.
                A[2][5] = dt / 2.
                B[0][0] = dt / 2.
                B[1][1] = dt / 2.
                B[2][2] = dt / 2.
                #B[0][0] = 0.
                #B[1][1] = 0.
                #B[2][2] = 0.

                z = np.array(obs[i])
                #print(i, len(estimates), len(covariances), len(U))
                if myid ==0 and i == 0:
                    x, P = kalman.step_known(estimates[i], covariances[i], U[i], z, B=B, A=A, pr=True)
                else:
                    x, P = kalman.step_known(estimates[i], covariances[i], U[i], z, B=B, A=A)
                estimates[i] = x
                covariances[i] = P

                PI = pinv(P)
                q.append(np.dot(weight_matrix[myid][myid], np.dot(PI, x)))
                W.append(np.dot(weight_matrix[myid][myid], PI))

            # Publish q & W
            state_information.q = np.array(q).flatten()
            state_information.W = np.array(W).flatten()
            state_information.id = myid

            # Wait for # of msgs to be num_trackers
            information_pub.publish(state_information)
            while not all([v for k, v in irec.items()]):
                information_pub.publish(state_information)

            # Fuse q & W
            W = np.array(W)
            q = np.array(q)
            nmw = weight_matrix[myid]
            for i in edges:
                if information_W.get(i) is None:
                    print("Key Info", myid, i, information_W.keys())
                    print("IW", information_W)
            for i in edges:
                for j in range(num_targets):
                    #W[j] = W[j] + np.dot(nmw[i], information_W[i][j])
                    #q[j] = q[j] + np.dot(nmw[i], information_q[i][j])
                    W[j] = W[j] + information_W[i][j]
                    q[j] = q[j] + information_q[i][j]
            information_W = {}
            information_q = {}

            for i in range(num_targets):
                OI = pinv(W[i])
                for l in range(len(OI)):
                    for j in range(len(OI[l])):
                        if OI[l][j] < 0.:
                            OI[l][j] = 0.

                estimates[i] = np.dot(OI, q[i])
                covariances[i] = OI

            desired_pose.pose.position.x = estimates[0][0] + offset[0]
            desired_pose.pose.position.y = estimates[0][1] + offset[1]
            desired_pose.pose.position.z = max(estimates[0][2], 3.) + offset[2]
            des_pub.publish(desired_pose)

            for e in edges:
                irec[e] = False
            pass

        rate.sleep()


if __name__ == "__main__":
    try:
        init_params()
        init_services()
        init_messages()
        track()
    except rospy.ROSInterruptException:
        pass
