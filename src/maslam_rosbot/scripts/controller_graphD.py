#!/usr/bin/env python3

# jplaced@unizar.es
# 2022, Universidad de Zaragoza

# This node recieve target exploration goals, which are the filtered frontier
# points published by the filter node, and commands the robot accordingly. The
# assigner node commands the robot through the move_base_node.

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Include modules~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
import rospy
import networkx as nx
import numpy as np
import heapq
import matplotlib.pyplot as plt

from constants import GRAPH_PATH_, PLAN_POINT_TH_, EXPLORING_TIME_, USE_GPU_, SHOW_DEBUG_PATH_, ODOM_COV_
from copy import copy, deepcopy
from functions import robot, wait_enterKey, getGraph
from weighted_pose_graph_class import weighted_pose_graph
from maslam_rosbot.msg import PointArray, InfoMatrix
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Bool
from actionlib_msgs.msg import GoalStatusArray

if USE_GPU_:
    from functions import cellInformation_NUMBA
else:
    from functions import cellInformation

from nav_msgs.msg import OccupancyGrid, Path

ODOM_FIM_ = np.linalg.inv(ODOM_COV_) / 2

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Callbacks~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
map_data_ = OccupancyGrid()
frontiers_ = []

# Variable to check the recevied merged frontiers
rec_centroids_flag = False
# Variable to check the recevied goal
rec_goal_flag = False


def frontiersCallBack(data):
    global frontiers_
    frontiers_ = []
    for point in data.points:
        frontiers_.append(np.array([point.x, point.y]))


def mapCallBack(data):
    global map_data_
    map_data_ = data


def mergedpointsCallback(data):
    global merged_frontiers_, rec_centroids_flag
    if not rec_centroids_flag:
        merged_frontiers_ = [np.array([point.x, point.y])
                             for point in data.points]
        rec_centroids_flag = True


def goalCallback(data):
    global rec_goal_flag, goal_coordinates
    goal_coordinates = []
    if not rec_goal_flag:
        goal_coordinates = data.points
        rec_goal_flag = True


def goalStatusCallback(msg):
    global status
    for goal_status in msg.status_list:
        status = goal_status.status


def serverBusyCallback(msg):
    global server_status
    server_status = msg.data


# Prototype of the callbacks used in an iterative way
def cncallbackReachedGoal(data, robot_id):
    global arrAck
    if not arrAck[robot_id]:
        arrAck[robot_id] = data.data


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Functions~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Function to create callbacks and subscriber in a iterative way
# depending on the number of robots used
def create_callbacks(num_robots):
    callbacks = []
    for i in range(num_robots):
        def callback(data, i=i): return cncallbackReachedGoal(data, i)
        callbacks.append(callback)

    return callbacks


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Hallucinates graph along the path to a frontier and computes its utility
def hallucinateGraph(G, p_frontier, info_radius):
    # Initialize hallucinated graph
    G_frontier = weighted_pose_graph(namespace=_namespace)
    G_frontier.copy_graph(G.graph)
    n = G.get_no_nodes()

    failed_hallucination = False

    # Pose of last known node
    temp = nx.get_node_attributes(G.graph, 'pose')
    p_last = np.array(temp[n - 1])

    # Hallucinate path
    plan = Path()
    plan = robot_.makePlan(robot_.getPosition(), p_frontier)
    n_points = len(plan)

    if n_points > 1:
        # Add new nodes & edges along the hallucinated path to the new frontier
        plan_nodes = np.ceil(n_points / PLAN_POINT_TH_)
        new_nodes = np.sort(np.random.choice(
            np.arange(1, n_points - 2), int(plan_nodes), replace=False))
        # Add one last node at frontier's location
        new_nodes = np.append(new_nodes, n_points-1)

        id_last = id_new = n - 1
        last_known_Info = G_frontier.graph.edges([id_last], 'information')

        # wait_enterKey()
        for i in new_nodes:
            p_path = np.array(
                [plan[i].pose.position.x, plan[i].pose.position.y])
            th_path = np.arctan2(p_path[1] - p_last[1], p_path[0] - p_last[0])

            id_new += 1
            G_frontier.graph.add_node(id_new, pose=p_path, theta=th_path)

            last_Info = G_frontier.graph.edges([id_last], 'information')

            try:
                if list(last_Info)[0] and len(list(last_Info)[0]) == 3:
                    """
                    I = np.array(list(last_Info)[0][2])
                    last_FIM = [[I[0], I[1], I[2]],
                               [I[1], I[3], I[4]],
                               [I[2], I[4], I[5]]]
                    new_cov = np.linalg.inv(last_FIM) + ODOM_COV_
                    new_FIM = np.linalg.inv(new_cov)
                    """
                    # Account for the expected unknown region that will be seen in the hallucinated path
                    if USE_GPU_:
                        normalized_unk_region_info_i, LC_info_i = cellInformation_NUMBA(np.array(map_data_.data),
                                                                                        map_data_.info.resolution,
                                                                                        map_data_.info.width,
                                                                                        map_data_.info.origin.position.x,
                                                                                        map_data_.info.origin.position.y,
                                                                                        p_path[0], p_path[1], info_radius)
                    else:
                        normalized_unk_region_info_i, LC_info_i = cellInformation(map_data_,
                                                                                  [p_path[0],
                                                                                      p_path[1]],
                                                                                  info_radius)

                    # Remove the node whose frontier is out the mapped
                    if normalized_unk_region_info_i != np.inf and LC_info_i != np.inf:
                        # Account for the expected unknown region that exists in that frontier
                        new_FIM = ODOM_FIM_ * (1 + normalized_unk_region_info_i)

                        G_frontier.addEdge(id_last, id_new, 0, [new_FIM[0][0], new_FIM[0][1], new_FIM[0][2],
                                                                new_FIM[1][1], new_FIM[1][2],
                                                                new_FIM[2][2]])

                        # print("od_info: " + format(1 + 2*normalized_unk_region_info_i))

                        # Account for LC
                        LC_candidates = G_frontier.find_overlapping_neighbors(
                            id_new, 2.0)
                        if LC_info_i > 0.03:  # If there is any structure in the region to close loops
                            I = np.array(list(last_known_Info)[0][2])
                            # I2 = np.array(list(last_Info)[0][2])
                            for j in LC_candidates:
                                # if np.random.random() > LC_info_i:
                                loop_diff = np.abs(j-id_new) / \
                                    G_frontier.get_no_nodes()

                                j_FIM = G_frontier.graph.edges([j], 'information')
                                I_j = np.array(list(j_FIM)[0][2])
                                FIM_LC = np.abs(I_j-I) * \
                                    (0.+1.5*(loop_diff*LC_info_i))
                                # FIM_LC = I * (loop_diff*LC_info_i)

                                G_frontier.addEdge(j, id_new, 1, FIM_LC)

                        # Update variables
                        id_last = id_new
                        p_last = p_path
                else:
                    rospy.logwarn("Error in Information matrix - Check graph.")
            except IndexError:
                pass

        # Save points along path as MarkerArray for visualization purposes
        if SHOW_DEBUG_PATH_:
            marker_hallucinated_graph_pub_.publish(
                G_frontier.getGraphAsMarkerArray())
            fig, ax = plt.subplots()
            ax.spy(G_frontier.compute_L(), precision=0, alpha=1, markersize=3)
            ax.spy(G.compute_L(), precision=0,
                   color='r', alpha=1, markersize=3)
            plt.show()
            wait_enterKey()

    else:  # Void returns if no points in path, most times due to frontiers lying in high potential area of cost map
        failed_hallucination = True
        rospy.logerr(
            f"{rospy.get_name()}: No points in plan to frontier at {format(p_frontier)}. Probably a high potential area!!"
        )

    return G_frontier, failed_hallucination

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Node~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


def node():
    global frontiers_, merged_frontiers_, map_data_, robot_, rec_centroids_flag, rec_goal_flag, goal_coordinates, arrAck, status, server_status
    if SHOW_DEBUG_PATH_:
        global marker_hallucinated_graph_pub_

    # List for the merged frontiers
    merged_frontiers_ = []
    # List for the agents' goal
    goal_coordinates = []
    # Variable to take the goal status of the agent
    status = -1
    # Variable to take the server status
    server_status = False

    rospy.init_node('assigner', anonymous=False)

    map_topic = rospy.get_param('~map_topic', '/map')
    namespace = rospy.get_param('~robot_ns')
    global _namespace
    _namespace = namespace
    info_radius = rospy.get_param('~info_radius', 1.0)
    frontiers_topic = rospy.get_param('~frontiers_topic', 'filtered_points')
    rateHz = rospy.get_param('~rate', 1)
    num_robots = rospy.get_param('~num_robots')
    delay_after_assignment = rospy.get_param('~delay_after_assignment', 0.5)

    rate = rospy.Rate(rateHz)
    rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
    rospy.Subscriber(frontiers_topic, PointArray, frontiersCallBack)

    # Creating the callbacks and the subcribers
    callbacks = create_callbacks(num_robots)
    # Create boolean variables to keep track of the robot reached or not the goal
    # Initialised to True since there is no goal to reach at the starting point of the program
    arrAck = [True] * num_robots

    # Create subscribers for each robot
    for i in range(num_robots):
        topic_name = f"/robot_{i}/reached_goal"
        message_type = Bool
        rospy.Subscriber(topic_name, message_type, callbacks[i])

    # Subscriber to retrieve the list of the merged points
    rospy.Subscriber("/merged_centroids", PointArray, mergedpointsCallback)
    # Subscriber to retrieve the list of the goals
    rospy.Subscriber("/chosen_points", PointArray, goalCallback)
    # SUbscriber to retrieve the result of the move base goal
    rospy.Subscriber("move_base/status", GoalStatusArray, goalStatusCallback)
    # SUbscriber to know if the central server is busy or not
    rospy.Subscriber("/central_sever_status", Bool, serverBusyCallback)

    # Publisher to pass the centroid of each agent to the manager node
    agent_centroids_publisher = rospy.Publisher(
        'agent_centroid', PointArray, queue_size=10)

    # Publisher to pass the matix [Reward, X, Y] to the manager node
    matrix_publisher = rospy.Publisher(
        'reward_information', InfoMatrix, queue_size=10)

    # Publisher to store if an agent has already reached the goal or not
    goal_reached_publisher = rospy.Publisher(
        'reached_goal', Bool, queue_size=10)
    
    # Publisher to inform if an agent has to get closer to a goal
    closer_goal_publisher = rospy.Publisher(
        'closer_goal', Bool, queue_size=10)

    if SHOW_DEBUG_PATH_:
        marker_hallucinated_graph_pub_ = rospy.Publisher(
            'marker_hallucinated_graph', MarkerArray, queue_size=10)

    # Wait if no frontier is received yet
    while len(frontiers_) < 1:
        pass

    # Wait if map is not received yet
    while len(map_data_.data) < 1:
        pass

    robot_name = namespace
    robot_ = robot(robot_name)
    robot_.sendGoal(robot_.getPosition())

    # Get ROS time in seconds
    t_0 = rospy.get_time()
    t_f_acc_decision_making = 0

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    while not rospy.is_shutdown():

        rospy.sleep(1)

        t_0_decision_making = rospy.get_time()

        # Print log
        rospy.loginfo('Waiting for server to be available')
        while server_status:
            continue

        # Wait for all agents to reach the goal
        # while not all(arrAck):
        #     # Publishing an ack to inform the goal has been reached
        #     msg = Bool()
        #     msg.data = True
        #     goal_reached_publisher.publish(msg)
        #     continue
        # # Reset the flag
        # arrAck = [False] * num_robots

        # Print log
        rospy.loginfo(f'{robot_name} processing points')

        # Coopy the frontiers once all agents reached the goal
        centroids = deepcopy(frontiers_)

        tempPointArray = PointArray()
        tempPoint = Point()
        tempPoint.z = 0.0

        # Reset the flag cause another previous publication may have triggered it
        rec_centroids_flag = False

        # Use the client to fuse the centroids and take the result
        for i in centroids:
            tempPoint.x = i[0]
            tempPoint.y = i[1]
            tempPointArray.points.append(copy(tempPoint))
        agent_centroids_publisher.publish(tempPointArray)

        rospy.logwarn('Published list for being merged')

        # Wait until the node recevied the merged frontiers
        while not rec_centroids_flag:
            continue
        # Reset the flag
        rec_centroids_flag = False

        # Print log
        centroids = merged_frontiers_
        rospy.loginfo(robot_name + ' received ' + str(len(merged_frontiers_)) + ' merged frontiers')

        # Replace the old centroids with the merged one
        centroids = merged_frontiers_

        originalLen = len(centroids)
        for ip in range(originalLen):
            i = ip - originalLen + len(centroids)
            if np.linalg.norm(robot_.getPosition() - centroids[i]) < 0.25:
                rospy.logwarn("Deleted a frontier too close to the robot.")
                del centroids[i]

        n_centroids = len(centroids)

        # Get SLAM graph
        nodes, edges = getGraph(GRAPH_PATH_+"graph_" + str(robot_name.split("_")[1]) +".g2o")
        G = weighted_pose_graph(namespace, nodes, edges, 'd_opt')

        # If no nodes (starting step) build graph with one edge at origin.
        n = float(G.get_no_nodes())
        m = float(G.get_no_edges())
        if n < 1:
            G.graph.add_node(0, pose=[0, 0], theta=0)

        infoGain = []
        closer_goal = False

        # If only one frontier no need to evaluate anything. Select that frontier
        if n_centroids == 1:
            rospy.logwarn("Only one frontier detected. Selecting it.")
            infoGain.append(np.ndarray.flatten(np.random.rand(1, 1)))
        # If no edges (starting step) do not evaluate D-opt. Select random frontier
        if m < 1 or centroids == 0:
            rospy.logwarn(
                "Graph not started yet, m < 1. Selecting goal +=[0.2,0.2].")
            closer_goal = True
            msg = Bool()
            msg.data = True
            closer_goal_publisher.publish(msg)
        else:  # Otherwise
            rospy.loginfo(
                f"{namespace}/controller_graph computing information gain of every frontier candidate."
            )
            for ip in range(n_centroids):
                # Get frontier goal
                p_frontier = np.array([centroids[ip][0], centroids[ip][1]])

                # Compute hallucinated pose graph
                G_frontier, flag = hallucinateGraph(G, p_frontier, info_radius)
                if flag:
                    rospy.logerr(
                        f"{rospy.get_name()}: No points in plan to frontier at {format(p_frontier)}. Assigning -Inf information!!"
                    )
                    infoGain.append(-np.inf)
                else:
                    # Compute no. of spanning trees <=> D-opt(FIM)
                    n_frontier = float(G_frontier.get_no_nodes())
                    L_anch = G_frontier.compute_anchored_L()
                    _, t = np.linalg.slogdet(L_anch.todense())
                    spann = n_frontier ** (1 / n_frontier) * \
                        np.exp(t / n_frontier)
                    infoGain.append(spann)

        if robot_.getState() == 1:
            t_f_acc_decision_making += rospy.get_time() - t_0_decision_making
            rospy.logwarn("Robot is not available.")
        elif closer_goal:
            t_f_acc_decision_making += rospy.get_time() - t_0_decision_making
            robot_.sendGoal(robot_.getPosition() + [0.2, 0.2])
        # The previous goal has been successfully reached
        else:
            infoGain_record = []
            centroid_record = []

            for ip in range(len(centroids)):
                infoGain_record.append(infoGain[ip])
                centroid_record.append(centroids[ip])

            # Find the max reward to print the reward normalised
            _max_ = np.max(infoGain_record)

            # if len(infoGain_record) <= 0:
            #    winner_id = 0
            # else:
            #    winner_id = infoGain_record.index(np.max(infoGain_record))

            t_f_acc_decision_making += rospy.get_time() - t_0_decision_making

            rospy.loginfo(namespace+"/controller_graph Information record: [Reward, X, Y] \n" +
                          format(np.column_stack((infoGain_record/_max_, centroid_record))))
            
            # Reset the flag cause another previous publication may have triggered it
            rec_goal_flag = False

            # Crete a matrix by putting the nx1 array and the nx2 matrix alongside
            matrix = np.column_stack((infoGain_record, centroid_record))
            matrix_msg = InfoMatrix()
            matrix_msg.rows = len(infoGain_record)
            # flatten the matrix and store the data in the message
            matrix_msg.data = matrix.flatten().tolist()

            # Publish the list on the topic
            matrix_publisher.publish(matrix_msg)

            # Wait until the goal is published
            while not rec_goal_flag:
                continue
            # Reset the flag
            rec_goal_flag = False

            print(str(robot_name) +" COORDINATES RECEIVED " + str(goal_coordinates))

            # Print log
            rospy.loginfo('Received goals. Extracting the correct one')

            # The goal position is the first for each robot cause the server now
            # gives one goal after each execution casue only one robot at time 
            # is using it
            selected_point = goal_coordinates[0]

            # Fill the arrat for the goal coordinates
            _goal_ = []
            _goal_.append(selected_point.x)
            _goal_.append(selected_point.y)

            rospy.loginfo(f"{robot_name} assigned to {str(_goal_)}")
            initial_plan_position = robot_.getPosition()
            robot_.sendGoal(_goal_)

            # Print log
            rospy.loginfo(f"{robot_name} reached {str(_goal_)}")

            # If plan fails near to starting position, send new goal to the next best frontier
            if robot_.getState() != 3 and n_centroids != 1:
                norm = np.linalg.norm(
                    robot_.getPosition() - initial_plan_position)
                if norm <= 2.0:
                    try:
                        second_max = heapq.nlargest(2, infoGain_record)[1]
                        winner_id = infoGain_record.index(second_max)
                        rospy.logwarn(
                            f"Goal aborted near previous pose (eucl = {str(norm)}). Sending new goal to: {str(_goal_)}"
                        )
                        robot_.sendGoal(_goal_)
                    finally:
                        pass
                else:
                    rospy.logwarn(
                        f"Goal aborted away from previous pose (eucl = {str(norm)}). Recomputing."
                    )

            rospy.sleep(delay_after_assignment)

        t_f = rospy.get_time() - t_0  # Get ROS time in seconds
        rospy.loginfo(
            (
                (
                    f"Decision making accumulated time: {format(t_f_acc_decision_making)} [sec]"
                    + " \n Total consumed time: "
                )
                + format(t_f)
                + " [sec]"
            )
        )
        if t_f >= EXPLORING_TIME_:
            wait_enterKey()
        rate.sleep()


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Main~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
