#!/usr/bin/env python

import sys
import rospy
import networkx as nx
import numpy as np
from math import cos, sin
from scipy.linalg import block_diag
from geometry_msgs.msg import PoseStamped
from resl_coverage.srv import Topology, TopologyRequest
from resl_coverage.srv import ProcessNoise, ProcessNoiseRequest
from resl_coverage.srv import Failure, FailureRequest
from resl_coverage.srv import Coordinates, CoordinatesRequest
from resl_coverage.srv import State, StateRequest
from resl_coverage.msg import StateEstimate

#from optimization_utils_dkf import *
#from reconfig_utils_dkf import *

num_targets = int(sys.argv[1])
num_trackers = int(sys.argv[2])

# Declare Service Nodes
process_services = []
failure_services = []
topology_services = []
des_offset_services = []
state_services = []

# Declare Service Objects
process_req = None
failure_req = None
topology_req = None
des_offset_req = None
state_req = None

# Declare Subscribers
tracker_subs = []
target_sub = None

# Declare Publishers
state_pub = None

# Declare Message Objects
tracker_poses = {}
states = {}
target = None


# Declare Graph Topology Data
received = 0
G = nx.Graph()
weight_matrix = np.zeros((num_trackers, num_trackers))
node_weights = {}

# Declare State Estimate Data
node_state_estimates = {}
node_covariances = {}
node_omegas = {}


def compute_offsets(coords):
    global target
    x = coords[0] - target.pose.position.x
    y = coords[1] - target.pose.position.y
    z = coords[2] - target.pose.position.z
    return [x, y, z]

def target_callback(msg):
    global target
    target = msg


def tracker_pose_callback(msg, arg):
    global tracker_poses
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z
    tracker_poses[arg] = [x, y, z]


def init_services():
    global process_req, failure_req, topology_req
    global state_req, des_offset_req
    global process_services, failure_services, topology_services
    global des_offset_services, state_services
    global num_trackers
    process_req = ProcessNoiseRequest()
    failure_req = FailureRequest()
    topology_req = TopologyRequest()
    des_offset_req = CoordinatesRequest()
    state_req = StateRequest()

    for i in range(num_trackers):
        rospy.wait_for_service('/tracker'+str(i)+'/topology')
        rospy.wait_for_service('/tracker'+str(i)+'/process_noise')
        rospy.wait_for_service('/tracker'+str(i)+'/state_estimate')
        rospy.wait_for_service('/tracker'+str(i)+'/desired_offsets')

        topology_services.append(
                rospy.ServiceProxy('/tracker'+str(i)+'/topology', Topology))
        des_offset_services.append(
                rospy.ServiceProxy('/tracker'+str(i)+'/desired_offsets', 
                    Coordinates, persistent=True))
        process_services.append(
                rospy.ServiceProxy('/tracker'+str(i)+'/process_noise',
                    ProcessNoise, persistent=True))
        failure_services.append(
                rospy.ServiceProxy('/tracker'+str(i)+'/failure', Failure))
        state_services.append(
                rospy.ServiceProxy('/tracker'+str(i)+'/state_estimate',
                    State, persistent=True))


def init_messages():
    global tracker_subs, state_pub, target_sub
    global num_trackers, num_targets

    target_sub = rospy.Subscriber('/unity_command/target0/TrueState/pose', 
            PoseStamped, target_callback)
    for i in range(num_trackers):
        tracker_subs.append(
                rospy.Subscriber('/unity_command/tracker'+str(i)+'/TrueState/pose',
                    PoseStamped, tracker_pose_callback, i))

    state_pub = rospy.Publisher('/target_estimates', StateEstimate, queue_size=2*num_targets)


def gen_graph():
    global G, weight_matrix, node_weights
    global num_trackers

    for i in range(num_trackers):
        self_degree = G.degree(i)
        weights = {}
        neighbor_weights_total = 0.
        for n in list(G.neighbors(i)):
            degree = G.degree(n)
            mw = 1. / (1. + max(self_degree, degree))
            neighbor_weights_total += mw
            weights[n] = mw
            weight_matrix[i][n] = mw
        weights[i] = 1. - neighbor_weights_total
        node_weights[i] = weights
        weight_matrix[i][i] = 1. - neighbor_weights_total


def monitor():
    global received, node_weights, weight_matrix, G
    global num_targets, num_trackers
    global node_state_estimates, node_covariances, node_omegas
    global tracker_poses, state_pub, target
    global process_req, failure_req, topology_req, des_offset_req
    global state_services, state_req, des_offset_req
    global process_services, failure_services
    global des_offset_services, topology_services

    for i in range(1, num_trackers):
        G.add_edge(i - 1, i)
    gen_graph()

    # Send initial Graph Data
    topology_req.weight_matrix = np.array(weight_matrix).flatten()
    for i in range(num_trackers):
        topology_req.edges = list(G.neighbors(i))
        topology_services[i](topology_req)

    r = 10.
    c = [0., 0.]
    a = 15.
    while not target:
        pass
    for i in range(num_trackers):
        interval = (2 * 3.141592654) / num_trackers
        points = [r * cos(interval * i), r * sin(interval * i), a]
        nc = compute_offsets(points)
        des_offset_req.x = nc[0]
        des_offset_req.y = nc[1]
        des_offset_req.z = nc[2]
        des_offset_services[i](des_offset_req)

    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        states = {}
        for i in range(num_trackers):
            state = state_services[i](state_req)
            states[i] = np.array(state.state).reshape((num_targets, 4))

        SE = StateEstimate()
        state = [[0. for i in range(4)] for i in range(num_targets)]
        for i in range(num_targets):
            for j in range(num_trackers):
                state[i][0] += states[j][i][0]
                state[i][1] += states[j][i][1]
                state[i][2] += states[j][i][2]
                state[i][3] += states[j][i][3]
            SE.state[0] = state[i][0] / num_trackers
            SE.state[1] = state[i][1] / num_trackers
            SE.state[2] = state[i][2] / num_trackers
            SE.state[3] = state[i][3] / num_trackers
            SE.id = i
            state_pub.publish(SE)

        rate.sleep()


if __name__ == "__main__":
    try:
        rospy.init_node('base_station')
        init_services()
        init_messages()
        monitor()
    except rospy.ROSInterruptException:
        pass
