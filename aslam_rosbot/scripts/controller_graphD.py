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

from copy import deepcopy

from constants import GRAPH_PATH_, PLAN_POINT_TH_, EXPLORING_TIME_, USE_GPU_, SHOW_DEBUG_PATH_, ODOM_COV_

from functions import robot, wait_enterKey, getGraph, compute_entropy,createMarker, draw_marker
from weighted_pose_graph_class import weighted_pose_graph
from aslam_rosbot.msg import PointArray
from geometry_msgs.msg import PoseStamped, Pose, Point

from visualization_msgs.msg import Marker, MarkerArray
import matplotlib.pyplot as plt
from aslam_rosbot.msg import SinglePath
from aslam_rosbot.msg import FrontierWithPath

if USE_GPU_:
    from functions import cellInformation_NUMBA
else:
    from functions import cellInformation

from nav_msgs.msg import OccupancyGrid, Path


ODOM_FIM_ = np.linalg.inv(ODOM_COV_) / 2

forntier_ID=0
numoffrontiers =0

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Callbacks~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
map_data_ = OccupancyGrid()
frontiers_ = []
infogain_frontiers = FrontierWithPath()


def info_frontiersCallBack(data):
    global infogain_frontiers
    infogain_frontiers = data
    rospy.loginfo("frontier No. {}".format(infogain_frontiers.ID))
    rospy.loginfo("has spanning trees. {}".format(infogain_frontiers.spanning_trees))
    rospy.loginfo("and entropy {}".format(infogain_frontiers.entropy))
    rospy.loginfo("with infogain {}".format(infogain_frontiers.infogain))        

def frontiersCallBack(data):
    global frontiers_
    frontiers_ = []
    for point in data.points:
        frontiers_.append(np.array([point.x, point.y]))


def mapCallBack(data):
    global map_data_
    map_data_ = data
    #rospy.loginfo("I got the Map data as ={}".format(data.data))
    # occupancy likelihood
    # 0 = free
    # 100 = occupied
    # -1 = unknown

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Functions~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Hallucinates graph along the path to a frontier and computes its utility
def hallucinateGraph(G, p_frontier, info_radius):

    global forntier_ID, numoffrontiers, infogain_frontiers

    #rospy.loginfo("   ==============================frontier ID = {} +++++++++++++++++++++".format(forntier_ID))

    # Initialize hallucinated graph
    G_frontier = weighted_pose_graph()
    G_frontier.copy_graph(G.graph)
    n = G.get_no_nodes()

    failed_hallucination = False

    # Pose of last known node
    temp = nx.get_node_attributes(G.graph, 'pose')
    p_last = np.array(temp[n - 1])

    # Hallucinate path
    plan = Path()
    plan = robot_.makePlan(robot_.getPosition(), p_frontier)

    n_points = int(len(plan))

    if n_points > 1:
        # Add new nodes & edges along the hallucinated path to the new frontier
        plan_nodes = np.ceil(n_points / PLAN_POINT_TH_)
        new_nodes = np.sort(np.random.choice(np.arange(1, n_points - 2), int(plan_nodes), replace=False))
        new_nodes = np.append(new_nodes, n_points-1)  # Add one last node at frontier's location

        id_last = id_new = n - 1
        last_known_Info = G_frontier.graph.edges([id_last], 'information')

        # wait_enterKey()
        for i in new_nodes:
            p_path = np.array([plan[i].pose.position.x, plan[i].pose.position.y])
            th_path = np.arctan2(p_path[1] - p_last[1], p_path[0] - p_last[0])  
            #atan2(y,x) gives you the angle between the x-axis and the vector (x,y),           

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
                                                                                  [p_path[0], p_path[1]],
                                                                                  info_radius)

                    # Account for the expected unknown region that exists in that frontier
                    new_FIM = ODOM_FIM_ * (1 + normalized_unk_region_info_i)

                    G_frontier.addEdge(id_last, id_new, 0, [new_FIM[0][0], new_FIM[0][1], new_FIM[0][2],
                                                            new_FIM[1][1], new_FIM[1][2],
                                                            new_FIM[2][2]])

                    # print("od_info: " + format(1 + 2*normalized_unk_region_info_i))

                    # Account for LC
                    LC_candidates = G_frontier.find_overlapping_neighbors(id_new, 2.0)
                    if LC_info_i > 0.03:  # If there is any structure in the region to close loops
                        I = np.array(list(last_known_Info)[0][2])
                        # I2 = np.array(list(last_Info)[0][2])
                        for j in LC_candidates:
                            rospy.loginfo("found loop closure candidate")
                            # if np.random.random() > LC_info_i:
                            loop_diff = np.abs(j-id_new)/G_frontier.get_no_nodes()  # normalized
                            j_FIM = G_frontier.graph.edges([j], 'information')
                            I_j = np.array(list(j_FIM)[0][2])

                            FIM_LC = np.abs(I_j-I) * (0.+1.5*(loop_diff*LC_info_i))
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
            marker_hallucinated_graph_pub_.publish(G_frontier.getGraphAsMarkerArray())
            #fig, ax = plt.subplots()
            fig, ax = plt.subplots(1, 2, figsize=(10, 5))
            ax[0].spy(G_frontier.compute_L(), color='b', precision=0, alpha=1, markersize=3) #laplacian of the weighted pose graph of each frontier
            x[0].set_title('Frontier laplacian')
            ax[1].spy(G.compute_L(), precision=0, color='r', alpha=1, markersize=3) # laplacian of the weighted pose graph fro+m the g2o map file ?
            ax[1].set_title('Graph laplacian')
            plt.show()
            wait_enterKey()

    else:  # Void returns if no points in path, most times due to frontiers lying in high potential area of cost map
        failed_hallucination = True
        rospy.logerr(rospy.get_name() + ": No points in plan to frontier at " + format(p_frontier) +
                     ". Probably a high potential area!!")

    return G_frontier, failed_hallucination

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Node~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
def node():
    global frontiers_, map_data_, robot_, f_path_pub_, forntier_ID, numoffrontiers ,infogain_frontiers,markerArray_pu,marker_pub   

    if SHOW_DEBUG_PATH_:
        global marker_hallucinated_graph_pub_

    rospy.init_node('assigner', anonymous=False)

    map_topic = rospy.get_param('~map_topic', '/map')
    info_radius = rospy.get_param('~info_radius', 1.0)
    frontiers_topic = rospy.get_param('~frontiers_topic', '/filtered_points')
    namespace = rospy.get_param('~namespace', '')
    rateHz = rospy.get_param('~rate', 1)
    namespace_init_count = rospy.get_param('namespace_init_count', 1)
    delay_after_assignment = rospy.get_param('~delay_after_assignment', 0.5)
   
    rate = rospy.Rate(rateHz)
    rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
    rospy.Subscriber(frontiers_topic, PointArray, frontiersCallBack)
     
    
    markerArray_pub = rospy.Publisher("visualization_markerArray/EntropyPath2", MarkerArray, queue_size=10)    
    marker_pub = rospy.Publisher("visualization_marker/EntropyPath", Marker, queue_size=10)


    if SHOW_DEBUG_PATH_:
        marker_hallucinated_graph_pub_ = rospy.Publisher('marker_hallucinated_graph', MarkerArray, queue_size=10)


    # Wait if no frontier is received yet
    while len(frontiers_) < 1:
        pass

    # Wait if map is not received yet
    while len(map_data_.data) < 1:
        pass

    robot_name = namespace + str(namespace_init_count)
    robot_ = robot(robot_name)
    robot_.sendGoal(robot_.getPosition())
    

    # Get ROS time in seconds
    t_0 = rospy.get_time()
    t_f_acc_decision_making = 0


    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    while not rospy.is_shutdown():

        rospy.sleep(1) 
        t_0_decision_making = rospy.get_time()
        centroids = deepcopy(frontiers_)

        originalLen = len(centroids)
        for ip in range(0, originalLen):
            i = ip - originalLen + len(centroids)
            if np.linalg.norm(robot_.getPosition() - centroids[i]) < 0.25:
                rospy.logwarn("Deleted a frontier too close to the robot.")
                del centroids[i]

        n_centroids = len(centroids) # number of frontiers seen
 
        rospy.loginfo("I saw {} frontiers ".format(n_centroids))

        # Get SLAM graph
        nodes, edges = getGraph(GRAPH_PATH_) # reads from the .g2o  pose graph file 
        G = weighted_pose_graph(nodes, edges, 'd_opt') # make a weighted pose graph from .g2o file 
        #nx.draw(G, with_labels=True)
        #plt.savefig("graph.png")

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
        elif m < 1:
            rospy.logwarn("Graph not started yet, m < 1. Selecting goal +=[0.1,0.1].")
            closer_goal = True

        else:  # Otherwise
            #rospy.loginfo("Computing information gain of every frontier candidate.")
            for ip in range(0, n_centroids):  ##################### change this if only if you want to chage the frontier 
                # Get frontier goal
                p_frontier = np.array([centroids[ip][0], centroids[ip][1]])
                robotposxy = robot_.getPosition()                
                #entropy,markerArray, marker =  compute_entropy(map_data_,p_frontier[0],p_frontier[1],robotposxy) 

                #markerArray_pub.publish(markerArray) 
                #marker_pub.publish(marker)
                

                # Compute hallucinated pose graph 
                G_frontier, flag = hallucinateGraph(G, p_frontier, info_radius) # hallucinate the existing pose-graph towards every frontier candidate 
               
                if flag:
                    rospy.logerr(rospy.get_name() + ": No points in plan to frontier at " + format(p_frontier) +
                                 ". Assigning -Inf information!!")
                    infoGain.append(-np.inf)
                else:
                    # Compute no. of spanning trees <=> D-opt(FIM)
                    n_frontier = float(G_frontier.get_no_nodes())
                    L_anch = G_frontier.compute_anchored_L() # compute anchored laplacian 
                    _, t = np.linalg.slogdet(L_anch.todense()) # Compute the sign and (natural) logarithm of the determinant of an array.
                    spann = n_frontier ** (1 / n_frontier) * np.exp(t / n_frontier) # equation 42 of the journal paper
                    rospy.loginfo("========= No. of Spanning trees of forntier = {} are {}".format(ip,spann))       

                    infoGain.append(spann)# - entropy*100)
                    rospy.loginfo("========= Info. gain  of forntier = {} is  {} \n\n".format(ip,infoGain[ip]))  

                    #rospy.loginfo("========= No. of Spanning trees of forntier = {} are {}".format(ip,spann))
                    #rospy.loginfo("=========+++++++++++++++++++++++++=========================\n\n")                                                                        

        if robot_.getState() == 1:
            t_f_acc_decision_making += rospy.get_time() - t_0_decision_making
            rospy.logwarn("Robot is not available.")
        elif closer_goal:
            t_f_acc_decision_making += rospy.get_time() - t_0_decision_making
            robot_.sendGoal(robot_.getPosition() + [0.1, 0.1])
        else:
            infoGain_record = []
            centroid_record = []

            for ip in range(0, len(centroids)):
                infoGain_record.append(infoGain[ip])
                centroid_record.append(centroids[ip])

            winner_id = infoGain_record.index(np.max(infoGain_record)) # winner frontier
            rospy.loginfo("========= Winner forntier ID is =  {}\n\n".format(winner_id))
            rospy.loginfo("================+++++++++++++++++++++++++=========================\n\n")


            t_f_acc_decision_making += rospy.get_time() - t_0_decision_making

            #rospy.loginfo("Information record: [Reward, X, Y] \n" +
                          #format(np.column_stack((infoGain_record, centroid_record))))
            rospy.loginfo(robot_name + " assigned to " + str(centroid_record[winner_id]))
            initial_plan_position = robot_.getPosition()

            winner_frontier_marker = draw_marker(centroid_record[winner_id][0],centroid_record[winner_id][1], [0.5,0.5,0.2],"square", 0.5)             
            marker_pub.publish(winner_frontier_marker)
          
            #rospy.loginfo("winner frontier loc is = {}".format(centroid_record[winner_id]))
            robot_.sendGoal(centroid_record[winner_id])

            # If plan fails near to starting position, send new goal to the next best frontier
            if robot_.getState() != 3 and n_centroids != 1:
                norm = np.linalg.norm(robot_.getPosition() - initial_plan_position)
                if norm <= 2.0:
                    try:
                        second_max = heapq.nlargest(2, infoGain_record)[1]
                        winner_id = infoGain_record.index(second_max)
                        rospy.logwarn("Goal aborted near previous pose (eucl = " + str(norm)
                                      + "). Sending new goal to: " + str(centroid_record[winner_id]))
                        robot_.sendGoal(centroid_record[winner_id])
                    finally:
                        pass
                else:
                    rospy.logwarn("Goal aborted away from previous pose (eucl = " + str(norm) + "). Recomputing.")

            rospy.sleep(delay_after_assignment)

        t_f = rospy.get_time() - t_0  # Get ROS time in seconds
        rospy.loginfo("Decision making accumulated time: " + format(t_f_acc_decision_making) + " [sec]" +
                      " \n Total consumed time: " + format(t_f) + " [sec]")
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
