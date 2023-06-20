#!/usr/bin/env python3

import rospy
import os
import matplotlib.pyplot as plt
import numpy as np

from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Empty

# Get the absolute path to the package
package_path = os.path.dirname(os.path.abspath(__file__))

# Retrieve the number of agents as parameter
num_robots = rospy.get_param('~num_robots', 2)

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Callbacks~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Function to store the map discovered by all the agents together
def map_occupation(msg):
    global merged_x, merged_y
    area = map_area(msg)

    # Get current time in seconds
    current_time = rospy.Time.now().to_sec()

    # Append the current time and area to the respective arrays
    merged_x.append(current_time)
    merged_y = np.append(merged_y, area)

# Function to store dynamically the map discovered by each of the agent
def cncallback0(msg, robot_id):
    global map_robot_x, map_robot_y
    discovered_cells = sum(1 for cell in msg.data if cell == 0)
    area = discovered_cells * (msg.info.resolution ** 2)

    # Get current time in seconds
    current_time = rospy.Time.now().to_sec()

    # Append the current time and area to the respective arrays
    map_robot_x[robot_id].append(current_time)
    map_robot_y[robot_id] = np.append(map_robot_y[robot_id], area)

# Callback to get the automatically generated map
def generated_map_callback(msg):
    global generated_map_area
    # Compute the total area of the environment
    generated_map_area = map_area(msg)

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Functions~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Function to create callbacks in an iterative way
# depending on the number of robots used
def create_callbacks(num_robots, cb):
    callbacks = []
    for i in range(num_robots):

        # Choosing the callback type
        if cb == 0:
            def callback(data, i=i): return cncallback0(data, i)

        callbacks.append(callback)

    return callbacks

# Function to compute the total amount of area discovered
def map_area(map):
    discovered_cells = sum(1 for cell in map.data if cell == 0)
    return discovered_cells * (map.info.resolution ** 2)

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Node~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
class PlotterNode(object):
    def __init__(self):
        global merged_x, merged_y, map_robot_x, map_robot_y

        rospy.init_node('plotter_node', anonymous=True)

        # Get initial time in seconds
        self.inital_time = rospy.Time.now().to_sec()

        # Subscriber for the merged map
        rospy.Subscriber('/map', OccupancyGrid, map_occupation)

        callbacks0 = create_callbacks(num_robots, 0)

        # Subscribers for single agent map
        for i in range(num_robots):
            topic_name = f"robot_{i}/map"
            message_type = OccupancyGrid
            rospy.Subscriber(topic_name, message_type, callbacks0[i])

        # Subscriber for the generated map
        rospy.Subscriber('/map2d', OccupancyGrid, generated_map_callback)

        # Arrays to store x-values (time) and y-values (area)
        merged_x = []
        merged_y = np.array([])

        # Arrays to store x-values (time) and y-values (area) for each robot
        map_robot_x = [[] for _ in range(num_robots)]
        map_robot_y = [np.array([]) for _ in range(num_robots)]

        rospy.on_shutdown(self.shutdown)

        # Spin the node to prevent it from exiting immediately
        rospy.spin()

    def plot_figure(self):
        global merged_x, merged_y, map_robot_x, map_robot_y, generated_map_area

        plt.figure(figsize=(16, 9))
        plt.subplot(2, 1, 1)
        # Plot the data
        plt.plot(np.array(merged_x)-self.inital_time, (merged_y/generated_map_area)*100, label='Percentage of area discovered over time')

        plt.xlabel('Time [s]')
        plt.ylabel('Percentage map discovered')
        plt.legend()
        plt.title('Merged map occupation over time')
        plt.grid(True)

        plt.subplot(2, 1, 2)
        # Plot the data for each robot
        for i in range(num_robots):
            plt.plot(np.array(map_robot_x[i])-self.inital_time, (map_robot_y[i]/generated_map_area)*100,
                     label=f'Percentage of map discovered over time (robot_{i})')

        plt.xlabel('Time [s]')
        plt.ylabel('Percentage map discovered')
        plt.legend()
        plt.title('Single agent map percentage over time')
        plt.grid(True)

        # Set the same x-axis limits for both subplots
        min_x = min(np.min(np.array(merged_x)-self.inital_time), np.min(np.concatenate(map_robot_x)-self.inital_time))
        max_x = max(np.max(np.array(merged_x)-self.inital_time), np.max(np.concatenate(map_robot_x)-self.inital_time))
        plt.xlim(min_x, max_x)

        # Set the same y-axis limits for both subplots
        min_y = min(np.min((np.array(merged_y)/generated_map_area)*100), np.min((np.array(map_robot_y)/generated_map_area)*100))
        max_y = max(np.max((np.array(merged_y)/generated_map_area)*100), np.max((np.array(map_robot_y)/generated_map_area)*100))
        plt.ylim(min_y, max_y)

        # Adjust the figure
        plt.subplots_adjust(left=0.05, bottom=0.05, right=0.95, top=0.95, wspace=0.2, hspace=0.4)

        # Check if the "figures" folder exists, and create it if it doesn't
        figure_path = os.path.join(package_path, '..', 'figures')
        if not os.path.exists(figure_path):
            os.makedirs(figure_path)

        # Read the current number from the config file
        # Construct the absolute path to the file
        filename = os.path.join(package_path, '..', 'config', 'number_figures.txt')
        with open(filename, 'r') as file:
            n = int(file.read())

        # Save image
        plt.savefig(f"{figure_path}/figure_{n}.png")

        # Save the updated number back to the file
        with open(filename, 'w') as file:
            file.write(str(n + 1))

        # Close the figure to release resources
        plt.close()

    def shutdown(self):
        rospy.loginfo("Shutting down the plotter node...")
        self.plot_figure()
        rospy.sleep(1)


if __name__ == '__main__':
    node = PlotterNode()
