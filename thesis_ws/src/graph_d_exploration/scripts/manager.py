#!/usr/bin/env python3

# s4636216@studenti.unige.it
# 2023, Università degli Studi di Genova, LS2N Ecole Centrale de Nantes

import rospy
import numpy as np

from copy import copy
from graph_d_exploration.msg import InfoMatrix, PointArray, MergePointsResult, ChooseGoalResult, BoolArray
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Int32, Float32
from client_merge import MergePointsClient
from client_choose import ChooseGoalClient
from actionlib_msgs.msg import GoalStatusArray
from constants import GOAL_SKIP_WAIT
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Callbacks~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# Prototype of the callbacks used in an iterative way
def cncallback0(data, robot_id):
    global c, recc
    # if not recc[robot_id]:
    c[robot_id] = []
    for point in data.points:
        c[robot_id].append(np.array([point.x, point.y]))
    recc[robot_id] = True


def cncallback1(data, robot_id):
    global m, recr
    if not recr[robot_id]:
        m = np.array(data.data).reshape(data.rows, 3)
        recr[robot_id] = True


def cncallback2(data, robot_id):
    global graph_started
    graph_started[robot_id] = data.data


def cncallback3(msg, robot_id):
    global goal_status
    for s in msg.status_list:
        goal_status[robot_id] = s.status


def cncallback4(msg, robot_id):
    global server_requested
    server_requested[robot_id] = msg


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Functions~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Function to create callbacks in a iterative way
# depending on the number of robots used
def create_callbacks(num_robots, cb):
    callbacks = []
    for i in range(num_robots):

        # Choosing the callback type
        if cb == 0:
            def callback(data, i=i): return cncallback0(data, i)
        elif cb == 1:
            def callback(data, i=i): return cncallback1(data, i)
        elif cb == 2:
            def callback(data, i=i): return cncallback2(data, i)
        elif cb == 3:
            def callback(data, i=i): return cncallback3(data, i)
        elif cb == 4:
            def callback(data, i=i): return cncallback4(data, i)

        callbacks.append(callback)

    return callbacks


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Node~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
def node():
    global c, m, recc, recr, graph_started, goal_status, server_requested

    rospy.init_node('manager', anonymous=False)
    rospy.loginfo('Started node')

    # Retrieve the number of agents as paramter
    num_robots = rospy.get_param('~num_robots')

    # Creating the callbacks and the subcribers
    callbacks0 = create_callbacks(num_robots, 0)
    callbacks1 = create_callbacks(num_robots, 1)
    callbacks2 = create_callbacks(num_robots, 2)
    callbacks3 = create_callbacks(num_robots, 3)
    callbacks4 = create_callbacks(num_robots, 4)

    # Create empty lists to store the point data for each robot
    c = [[] for _ in range(num_robots)]

    # Create empty numpy arrays to store the matrix data for each robot
    m = [np.zeros((1, 3)) for _ in range(num_robots)]

    # Create boolean variables to keep track of whether the point and matrix data have been received for each robot
    recc = [False] * num_robots
    recr = [False] * num_robots
    graph_started = [False] * num_robots
    # Variable to store the status of the goal of each robot
    goal_status = [-1] * num_robots

    # Variable to store the agents' requests to the server
    # False: not requested / in use
    # True: requested (pending)
    server_requested = [False] * num_robots

    # Variable to pass back to the agents the id of the robot that can use the server
    array_allow = [False] * num_robots
    # Array to store how many iterations the agent i has to wait.
    # if it is too much then the priority is given to it
    iterations_waited = [0] * num_robots

    # Create variables to keep track of the number of rows in the matrix data for each robot
    rows = [0] * num_robots

    # Create subscribers for each robot
    for i in range(num_robots):
        topic_name = f"robot_{i}/agent_centroid"
        message_type = PointArray
        rospy.Subscriber(topic_name, message_type, callbacks0[i])

    # Create subscribers for each robot
    for i in range(num_robots):
        topic_name = f"robot_{i}/reward_information"
        message_type = InfoMatrix
        rospy.Subscriber(topic_name, message_type, callbacks1[i])

    # Create subscribers for each robot
    for i in range(num_robots):
        topic_name = f"robot_{i}/graph_started"
        message_type = Bool
        rospy.Subscriber(topic_name, message_type, callbacks2[i])

    # Create subscribers for each robot
    for i in range(num_robots):
        topic_name = f"robot_{i}/move_base/status"
        message_type = GoalStatusArray
        rospy.Subscriber(topic_name, message_type, callbacks3[i])

    # Create subscribers for each robot
    for i in range(num_robots):
        topic_name = f"robot_{i}/server_request"
        message_type = Bool
        rospy.Subscriber(topic_name, message_type, callbacks4[i])

    client_mergeCentroids = MergePointsClient('merge_client')
    client_choosePoints = ChooseGoalClient('choose_client')

    merged_centroids_pub = rospy.Publisher(
        'merged_centroids', PointArray, queue_size=10)

    chosen_points_pub = rospy.Publisher(
        'chosen_points', PointArray, queue_size=10)

    central_server_occupied_pub = rospy.Publisher(
        'central_sever_status', BoolArray, queue_size=10)
    
    received_list_points_publisher = rospy.Publisher(
        'list_points', Int32, queue_size=10)

    # Publisher for the radius value
    radius_value_publisher = rospy.Publisher(
        'radius_value', Float32, queue_size=10)


    rate = 0.25

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    while not rospy.is_shutdown():
        
        # Check the status of the requests from the agents
        id = -1
        # Look for the first which is true
        for index, value in enumerate(server_requested):
            if value:
                if id == -1:
                    id = index
                iterations_waited[index] += 1

        idx = -1
        for index, value in enumerate(iterations_waited):
            if value >= GOAL_SKIP_WAIT:
                if idx == -1:
                    idx = index

        # if both indexes are -1 it means that there are not any robots requesting the server
        if idx == -1 and id == -1:
            rospy.loginfo(
            'Waiting for points to merge ...')
        else:
            index_robot = -1
            # if a robot has waited too much
            if idx != -1:
                index_robot = idx
                array_allow[idx] = True
                # Reset the server requested from the agent
                server_requested[idx] = False
                rospy.loginfo(
                    f"robot_{str(idx)} allowed to use the server. It got the priority after waiting"
                )
            else:
                array_allow[id] = True
                index_robot = id
                # Reset the server requested from the agent
                server_requested[id] = False
                rospy.loginfo(
                    f"robot_{str(id)} allowed to use the server. It has the priority"
                )

            # Setting the itreations waited number to 0 since the robot got the priority
            iterations_waited[index_robot] = 0

            # Reset the flags
            recc = [False] * num_robots

            # Publish True in the position of the robot that can use the server
            central_server_occupied_pub.publish(array_allow)

            # Wait until at least one of the agents published the frontiers
            while not any(recc):
                continue

            # Publish a False for the allow.
            array_allow = [False] * num_robots
            central_server_occupied_pub.publish(array_allow)

            rospy.loginfo("Server Busy")

            rospy.loginfo('Sending goal for merging')

            # Send goal and store the result
            result = MergePointsResult()
            c_all = []
            for i in range(num_robots):
                c_all.extend(c[i])

            msg = Int32()
            if c_all != []:
                msg = len(c_all)
            else:
                msg = 0
            received_list_points_publisher.publish(msg)
            result = client_mergeCentroids.send_goal(c_all)

            tempPointArray = PointArray()

            # Publish the result back to the assigner nodes
            for i in range(len(result.merged_points)):
                tempPoint = Point()
                tempPoint.z = 0.0
                tempPoint.x = result.merged_points[i].x
                tempPoint.y = result.merged_points[i].y
                tempPointArray.points.append(copy(tempPoint))

            # Reset the flags
            recr = [False] * num_robots

            # Print log
            rospy.loginfo('Publishing result')
            merged_centroids_pub.publish(tempPointArray.points)
            radius_value_publisher.publish(result.radius_used)

            # Reset the flags
            recc = [False] * num_robots

            if graph_started[index_robot]:

                rospy.loginfo(
                    f'Waiting for robot_{index_robot} to publish [Reward, X, Y] information')

                while not any(recr):
                    continue

                # Send goal and store the result
                result = ChooseGoalResult()

                # Store the matrix
                concatenated_matrix = m

                # Store the number of rows for each matrix
                rows = m.shape[0]
                # Print log
                rospy.loginfo('Sending goal')
                # Pass the goal
                result = client_choosePoints.send_goal(
                    concatenated_matrix.flatten().tolist(), rows)

                # Publish the result back to the assigner nodes
                # Form the PointArray structure to publish
                tempPointArray = PointArray()
                tempPoint = Point()
                tempPoint.z = 0.0

                for i in result.goals:
                    tempPoint.x = round(i.x, 2)
                    tempPoint.y = round(i.y, 2)
                    tempPointArray.points.append(copy(tempPoint))


                chosen_points_pub.publish(tempPointArray)
                rospy.loginfo(f'Published goal for robot_{index_robot}')

                # Reset the flags
                recr = [False] * num_robots

            else:
                rospy.loginfo(f"Graph robot_{str(index_robot)} not started yed. Waiting")

        rospy.sleep(rate)


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Main~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
