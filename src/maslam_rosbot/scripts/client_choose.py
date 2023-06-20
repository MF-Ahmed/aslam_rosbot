#!/usr/bin/env python3

# s4636216@studenti.unige.it
# 2023, Università degli Studi di Genova, LS2N Ecole Centrale de Nantes

# This node receives all the centroids information from each agent and computes
# the best choice for each one depending on the reward passed

import rospy
import actionlib

from maslam_rosbot.msg import ChooseGoalAction, ChooseGoalGoal, ChooseGoalResult, InfoMatrix


class ChooseGoalClient:
    def __init__(self, client_name):
        self.client_name_ = client_name
        self.client = actionlib.SimpleActionClient(
            '/choose_points', ChooseGoalAction)

    # Run when client accepts goal
    def goal_response_callback(self):
        rospy.loginfo(f'{self.client_name_} goal accepted')

    # Run when client sends feedback
    def feedback_callback(self, feedback_msg):
        rospy.loginfo(
            f'{self.client_name_} received feedback:  {feedback_msg.check}')

    # Run when client sends final result
    def get_result_callback(self, state, result):
        # Show log and exit node
        log_message = "Result:\n"
        for coord in result.goals:
            log_message += f"x: {coord.x}, y: {coord.y}\n"
        rospy.loginfo(log_message)

        # rospy.signal_shutdown("Shutting-down client node")

    def send_goal(self, flattened_matrices, rows):
        goal_msg = ChooseGoalGoal()
        matrix = InfoMatrix()
        goal_msg.client_id = self.client_name_
        goal_msg.matrix.data = flattened_matrices
        goal_msg.rows = rows

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
        result = ChooseGoalResult()
        result = self.client.get_result()
        return result
