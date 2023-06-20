#!/usr/bin/env python3

# s4636216@studenti.unige.it
# 2023, Università degli Studi di Genova, LS2N Ecole Centrale de Nantes

# This script is the server class. Its task is to take the centroids of all the
# agents, passed through a client, merge into a single list, delete the redundant
# points, if any, and pass the list back to the client

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Include modules~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
import rospy
import actionlib

from maslam_rosbot.msg import MergePointsAction, MergePointsResult, MergePointsFeedback
from constants import GRAPH_PATH_


class MergePointsServer:
    def __init__(self):
        self.num_clients = 0
        self.r = rospy.Rate(0.5)  # Hz
        self.server = actionlib.SimpleActionServer(
            'merge_points', MergePointsAction, self.execute_callback, False)
        # Start the server
        self.server.start()
        rospy.loginfo('Merge Points Server started')

    def execute_callback(self, goal):
        rospy.loginfo(
            f'Received {len(goal.points)} points from client {goal.client_id}')

        # Print log messages
        rospy.loginfo('Server received all points from client')
        result = MergePointsResult()
        result.merged_points = self.merge_points(goal.points)
        rospy.loginfo(
            f'Merged {len(result.merged_points)} points from all clients')
        self.server.set_succeeded(result)

        if self.server.is_preempt_requested():
            rospy.loginfo('Goal was preempted')
            self.server.set_preempted()
        elif rospy.is_shutdown():
            rospy.loginfo('Node was shutdown')
            self.server.set_aborted()

    def merge_points(self, list):
        rospy.loginfo('Merging points ...')
        feedback_msg = MergePointsFeedback()
        feedback_msg.percent_completed = 0

        # remove duplicate points with the same x and y coordinates
        unique_points = []
        for point in list:
            if point not in unique_points:
                unique_points.append(point)

        return unique_points


if __name__ == '__main__':
    rospy.init_node('merge_points_server')
    server = MergePointsServer()
    rospy.spin()
