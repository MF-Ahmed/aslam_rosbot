#!/usr/bin/env python3

# s4636216@studenti.unige.it
# 2023, Università degli Studi di Genova, LS2N Ecole Centrale de Nantes

# This node is the client class. Its task is to pass to and get from the server
# an np.array of points.

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Include modules~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
import sys

import numpy as np
import rospy
import actionlib
import contextlib

from maslam_rosbot.msg import Point2D, MergePointsAction, MergePointsGoal, MergePointsResult


class MergePointsClient:
    def __init__(self, client_name):
        self.client_name_ = client_name
        self.client = actionlib.SimpleActionClient(
            '/merge_points', MergePointsAction)
        # Keep track of the future objects
        self.future_handle = False

    # Run when client accepts goal
    def goal_response_callback(self):
        rospy.loginfo(f'{self.client_name_} goal accepted')

    # Run when client sends feedback
    def feedback_callback(self, feedback_msg):
        rospy.loginfo(self.client_name_ + ' received feedback: {0}'.format(
            feedback_msg.percent_completed))

    # Run when client sends final result
    def get_result_callback(self, state, result):
        # Show log and exit node
        rospy.loginfo(f'Received {len(result.merged_points)} points')

    # Waits for server to be available, then sends goal
    def send_goal(self, points_):
        points = np.array(points_)
        goal_msg = MergePointsGoal()
        goal_msg.client_id = self.client_name_

        for _point in points:
            p_ = Point2D()
            p_.x = _point[0]
            p_.y = _point[1]
            goal_msg.points.append(p_)

        rospy.loginfo(
            f'{self.client_name_} waiting for server...')
        self.client.wait_for_server()
        rospy.loginfo(f'{self.client_name_} sending goal...')

        # Returns future to goal handle; client runs feedback_callback after sending the goal
        self.future_handle = self.client.send_goal(
            goal_msg, active_cb=self.goal_response_callback, feedback_cb=self.feedback_callback, done_cb=self.get_result_callback)

        rospy.loginfo(f'{self.client_name_} goal sent!')
        # Block the code until this goal is completed
        rospy.loginfo(f'{self.client_name_} waiting for result')
        self.client.wait_for_result()

        rospy.loginfo(f'{self.client_name_} getting the result')

        # Take the result of the server
        result = MergePointsResult()
        result = self.client.get_result()
        return result


if __name__ == '__main__':
    with contextlib.suppress(rospy.ROSInterruptException):
        name = sys.argv[1]
        rospy.init_node(f'merge_client_{name}')
        client = MergePointsClient(f'merge_client_robot_{name}')

        pt = [
            np.array([18.47946582,  0.04982087]),
            np.array([20.72036815,  1.36476467]),
            np.array([20.12390291, -1.75186919]),
            np.array([24.36149457, -0.02550663])
        ]

        client.send_goal(pt)

        rospy.spin()
