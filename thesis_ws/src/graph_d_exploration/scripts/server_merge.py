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

import itertools
from graph_d_exploration.msg import MergePointsAction, MergePointsResult, MergePointsFeedback
from constants import RADIUS_BORDER_, PERCENTAGE_UNKNOWN_, NUMBER_POINTS_LIST_
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Callbacks~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Function to store the map discovered by all the agents together
def map_occupation(msg):
    global merged_map
    merged_map = msg


class MergePointsServer:
    def __init__(self):
        global merged_map

        # Variable to store the merged map until that moment
        merged_map = OccupancyGrid()

        self.num_clients = 0
        self.r = rospy.Rate(0.5)  # Hz
        self.server = actionlib.SimpleActionServer(
            'merge_points', MergePointsAction, self.execute_callback, False)

        # Subscriber for the merged map
        rospy.Subscriber('/map', OccupancyGrid, map_occupation)

        # Start the server
        self.server.start()
        rospy.loginfo('Merge Points Server started')

    def execute_callback(self, goal):
        rospy.loginfo(
            f'Received {len(goal.points)} points from client {goal.client_id}')

        # Print log messages
        rospy.loginfo('Server received all points from client')
        result = MergePointsResult()
        result.merged_points, result.radius_used = self.merge_points(goal.points)
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

        radius_ = RADIUS_BORDER_

        # remove duplicate points with the same x and y coordinates
        unique_points = []
        for point in list:
            # Check that the point is near a border and has enough
            # unknown cells in its nearby
            if self.is_near_border(point, merged_map, radius_):
                # Check that the point has not alrady been chosen
                if point not in unique_points:
                    unique_points.append(point)

        while(len(unique_points) >= NUMBER_POINTS_LIST_):
            rospy.logwarn(f'Recomputin list: currently {len(unique_points)} points.')
            _unique_points_ = unique_points
            unique_points = []
            # increase the radius to use fewer points
            radius_ += 0.25
            # Perform the computation with a higher percentage
            for point in _unique_points_:
                if self.is_near_border(point, merged_map, radius_):
                    if point not in unique_points:
                        unique_points.append(point)

        return unique_points, radius_

    def is_near_border(self, point, merged_map, radius):
        grid = merged_map.data
        if merged_map.info.resolution:
            resolution = merged_map.info.resolution
        else:
            resolution = 0.05
        map_width = merged_map.info.width
        map_height = merged_map.info.height

        # Obtain point coordinates
        point_x = point.x
        point_y = point.y

        origin_x = merged_map.info.origin.position.x
        origin_y = merged_map.info.origin.position.y
        cell_x = int((point_x - origin_x) / resolution)
        cell_y = int((point_y - origin_y) / resolution)

        # Convert radius to grid units
        radius_in_cells = int(radius / resolution)
        # Extract set of cells in the circumference
        cells_in_circumference = set()
        # Counter for unknown cells
        unknown_count = 0
        # Counter for total cells
        total_cells = 0

        for i in range(-radius_in_cells, radius_in_cells + 1):
            for j in range(-radius_in_cells, radius_in_cells + 1):
                cell_i = cell_x + i
                cell_j = cell_y + j
                if cell_i >= 0 and cell_i < map_width and cell_j >= 0 and cell_j < map_height:
                    index = cell_i + cell_j * map_width
                    cells_in_circumference.add((cell_i, cell_j))
                    if grid[index] == -1:
                        unknown_count += 1
                    total_cells += 1

        if total_cells != 0:
            # Calculate the percentage of unknown cells
            unknown_percentage = (unknown_count / total_cells) * 100

            # Check if the unknown percentage meets the desired threshold
            # and return the condition
            return unknown_percentage >= PERCENTAGE_UNKNOWN_

        return False

if __name__ == '__main__':
    rospy.init_node('merge_points_server')
    server = MergePointsServer()
    rospy.spin()
