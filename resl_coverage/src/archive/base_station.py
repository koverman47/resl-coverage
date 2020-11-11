#!/usr/bin/env python

import sys
import rospy
import networkx as nx
import numpy as np
from scipy.linalg import block_diag
from geometry_msgs.msg import PoseStamped
from resl_resilient_tracking.srv import Topology, TopologyRequest
from resl_resilient_tracking.srv import ProcessNoise, ProcessNoiseRequest
from resl_resilient_tracking.srv import Failure, FailureRequest
from resl_resilient_tracking.srv import Coordinates, CoordinatesRequest

from optimization_utils_dkf import *
from reconfig_utils_dkf import *

num_targets = int(sys.argv[1])
num_trackers = int(sys.argv[2])

# Declare Service Nodes
process_services = []
failure_services = []
topology_services = []
des_offset_services = []

# Declare Service Objects
process_req = None
failure_req = None
topology_req = None
des_offset_req = None

# Declare Subscriber Nodes
tracker_subs = []

# Declare Message Objects
tracker_poses = {}

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


def tracker_pose_callback(msg, arg):
    global tracker_poses
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z
    tracker_poses[arg] = [x, y, z]


def init_services():
    global process_req, failure_req, topology_req
    global in_coords_req, out_coords_req, des_offset_req
    global process_services, failure_services, topology_services
    global des_coordinates_services
    global num_trackers
    process_req = ProcessNoiseRequest()
    failure_req = FailureRequest()
    topology_req = TopologyRequest()
    des_offset_req = CoordinatesRequest()

    for i in range(num_trackers):
        rospy.wait_for_service('/tracker'+str(i)+'/topology')
        rospy.wait_for_service('/tracker'+str(i)+'/process_noise')

        topology_services.append(
                rospy.ServiceProxy('/tracker'+str(i)+'/topology', Topology))
        des_offset_services.append(
                rospy.ServiceProxy('tracker'+str(i)+'/desired_offsets', Coordinates))
        process_services.append(
                rospy.ServiceProxy('/tracker'+str(i)+'/process_noise',
                    ProcessNoise, persistent=True))
        failure_services.append(
                rospy.ServiceProxy('/tracker'+str(i)+'/failure', Failure))


def init_messages():
    global tracker_subs
    global num_trackers

    for i in range(num_trackers):
        tracker_subs.append(
                rospy.Subscriber('/unity_command/tracker'+str(i)+'/TrueState/pose',
                    PoseStamped, tracker_pose_callback, i))


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
    global tracker_poses
    global process_req, failure_req, topology_req, des_offset_req
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

    
    fail = (False, -1)
    quality = [10. for i in range(num_trackers)]
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        Rs = {}
        for i in range(num_trackers):
            rospy.wait_for_service('/tracker'+str(i)+'/process_noise')
            res = process_services[i](process_req)
            Rs[i] = np.array(res.R).reshape((6, 6))
            tr = np.trace(Rs[i])
            if tr <= quality[i]:
                quality[i] = tr
            else:
                print(i, tr, quality[i])
                quality[i] = tr
                fail = (True, i)

        if fail[0]:
            for i in range(num_trackers):
                res = failure_services[i](failure_req)
                node_state_estimates[i] = np.array(res.x).reshape((num_targets, 6))
                node_covariances[i] = np.array(res.P).reshape((num_targets, 6, 6))
                node_omegas[i] = np.array(res.W).reshape(num_targets, 6, 6)

            adj = nx.adjacency_matrix(G).todense()
            if not np.array_equal(np.diag(adj), np.ones(num_trackers)):
                adj = adj + np.diag(np.ones(num_trackers))
            cur_nw = node_weights

            # Stack cov_data, w_data diagonally (see new import)
            cov_data = {}
            w_data = {}
            for i in range(num_trackers):
                cov_data[i] = node_covariances[i][0]
                w_data[i] = node_omegas[i][0]
                for j in range(1, num_targets):
                    cov_data[i] = block_diag(cov_data[i], node_covariances[i][j])
                    w_data[i] = block_diag(w_data[i], node_omegas[i][j])
            cov_data = np.array([cov for n, cov in cov_data.items()])
            w_data = np.array([w for node, w in w_data.items()])


            #print("ADJ MAT", adj)
            #print("CURRENT WEIGHTS", cur_nw)
            #print("COV DATA", cov_data)
            print(cov_data.shape)
            print(cov_data)
            #print(w_data.shape)
            #print("W DATA", w_data)
            #print("FAILED NODE", fail[1])
            new_config, new_weights = team_opt_bnb(adj, cur_nw, cov_data, w_data, fail[1])
            #new_config, new_weights = team_opt_iter(adj, cur_nw, cov_data, w_data, fail[1])
            #print(new_config)

            G = nx.from_numpy_matrix(new_config)
            gen_graph()
            target_poses = {}
            for i in range(num_targets):
                choice = np.random.randint(0, num_trackers)
                target_poses[i] = node_state_estimates[choice][i]
            fov = {nid: 3 for nid in range(num_trackers)}
            #new_coords = generate_coords(new_config, tracker_poses, fov, target_poses)
            new_coords, sq = generate_coords(new_config, tracker_poses, fov, Rs)
            print(new_coords)

            for i in range(num_trackers):
                nc = compute_offsets(new_coords[i])
                des_offset_req.x = nc[0]
                des_offset_req.y = nc[1]
                des_offset_req.z = nc[2]
                des_offset_services[i](des_offset_req)
            print("Done Reconfiguration")
        rate.sleep()


if __name__ == "__main__":
    try:
        rospy.init_node('base_station')
        init_services()
        init_messages()
        monitor()
    except rospy.ROSInterruptException:
        pass
