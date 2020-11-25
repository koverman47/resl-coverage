#!/usr/bin/env python
import sys
import rospy
import numpy as np
import networkx as nx
from time import time
from kalman import Kalman
from numpy.linalg import pinv
from geometry_msgs.msg import PoseStamped, TwistStamped
from resl_coverage.msg import MultiStateEstimate
from resl_coverage.srv import Topology, TopologyResponse
from resl_coverage.srv import ProcessNoise, ProcessNoiseResponse
from resl_coverage.srv import Failure, FailureResponse
from resl_coverage.srv import Coordinates, CoordinatesResponse
from resl_coverage.srv import TriggerFail, TriggerFailResponse
from resl_coverage.srv import State, StateResponse

num_targets = int(sys.argv[1])
num_trackers = int(sys.argv[2])

# Declare Services
process_noise_res = None 
failure_res = None
des_offset_res = None
topology_res = None
trigger_fail_res = None
state_res = None

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
irec = {}
information_q = {} # Update to dictionary
information_W = {} # Update to dictionary
N = 1
edges = []
weight_matrix = None

obs = [[None, None, None, None] for i in range(num_targets)]
covariances = [np.dot(0.01, np.eye(4)) for i in range(num_targets)]
estimates = [np.array([0., 0., 0., 0.]) for i in range(num_targets)]
offset = [float(sys.argv[3]), float(sys.argv[4]), 5.]

A = None
B = None
H = None
U = None
Q = None
R = None

set_est = [[False, False] for i in range(num_targets)]
def pose_callback(msg, args):
    global obs, estimates, set_est, R
    noise = np.dot(R, np.random.random(4))
    obs[args][0] = msg.pose.position.x + (noise[0] * 0.01)
    obs[args][1] = msg.pose.position.y + (noise[1] * 0.01)
    if not set_est[args][0]:
        estimates[args][0] = msg.pose.position.x
        estimates[args][1] = msg.pose.position.y
        set_est[args][0] = True

def twist_callback(msg, args):
    global obs, estimates, set_est, R
    noise = np.dot(R, np.random.random(4))
    obs[args][2] = msg.twist.linear.x + (noise[2] * 0.01)
    obs[args][3] = msg.twist.linear.y + (noise[3] * 0.01)

    U[args][0] = msg.twist.linear.x
    U[args][1] = msg.twist.linear.y
    if not set_est[args][1]:
        estimates[args][2] = msg.twist.linear.x
        estimates[args][3] = msg.twist.linear.y
        set_est[args][1] = True

def offset_callback(msg):
    global offset
    offset[0] = msg.pose.position.x
    offset[1] = msg.pose.position.y
    offset[2] = msg.pose.position.z

def information_callback(msg):
    global irec, information_q, information_W
    global num_targets
    information_q[msg.id] = np.array(msg.q).reshape((num_targets, 4))
    information_W[msg.id] = np.array(msg.W).reshape((num_targets, 4, 4))
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
    offset[0] = req.x
    offset[1] = req.y
    offset[2] = req.z

    res = CoordinatesResponse()
    res.rec = 1
    return res

def handle_process_noise(req):
    global R
    return ProcessNoiseResponse(R.flatten())

def handle_failure(req):
    global estimates, covariances
    global consensus_subs, edges
    edges = []
    consensus_subs = []
    irec = {}


    res = FailureResponse()
    inter_W = np.array([pinv(covariances[i]) for i in range(num_targets)])
    res.W = inter_W.flatten()
    res.x = np.array(estimates).flatten()
    res.P = np.array(covariances).flatten()
    return res

def handle_trigger_fail(req):
    global R
    R = np.dot(1.1, R)
    res = TriggerFailResponse()
    return res

def handle_state_request(req):
    global estimates
    res = StateResponse()
    res.state = np.array(estimates).flatten()
    return res

def init_services():
    global name
    global process_noise_res, failure_res
    global des_offset_res, topology_res
    global trigger_fail_res, state_res

    topology_res = rospy.Service(name+'topology', Topology, handle_topology)
    des_offset_res = rospy.Service(name+'desired_offsets', Coordinates, handle_offsets)
    process_noise_res = rospy.Service(name+'process_noise', ProcessNoise, handle_process_noise)
    failure_res = rospy.Service(name+'failure', Failure, handle_failure)
    trigger_fail_res = rospy.Service(name+'trigger_fail', TriggerFail, handle_trigger_fail)
    state_res = rospy.Service(name+'state_estimate', State, handle_state_request)

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

    A = np.eye(4)
    B = np.concatenate((np.zeros((2, 2)), np.eye(2)))
    U = [np.ones(2) for i in range(num_targets)]
    Q = np.eye(4)
    H = np.eye(4)
    R = np.eye(4)

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
    desired_pose.pose.position.z = max(offset[2], 3.)
    
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
            # Update q & W
            for i in range(num_targets):
                z = np.array(obs[i])
                q.append(np.dot(weight_matrix[myid][myid], np.dot(H.T, np.dot(pinv(R), z))))
                W.append(np.dot(weight_matrix[myid][myid], np.dot(H.T, np.dot(pinv(R), H))))

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
                    W[j] = W[j] + information_W[i][j]
                    q[j] = q[j] + information_q[i][j]
            for i in range(num_targets):
                W[i] = np.dot(1. / N, W[i])
                q[i] = np.dot(1. / N, q[i])
            information_W = {}
            information_q = {}

            for i in range(num_targets):
                A[0][2] = dt
                A[1][3] = dt

                x, P = kalman.step_unknown(estimates[i], covariances[i], q[i], W[i], N+1, B=B, A=A)
                estimates[i] = x
                covariances[i] = P
 
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
