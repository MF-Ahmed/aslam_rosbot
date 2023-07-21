#!/usr/bin/env python3

import rospy
import os
import matplotlib.pyplot as plt
import numpy as np
import time
import psutil
import pynvml
import GPUtil
from graph_d_exploration.msg import PointArray
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Int32, Float32
from rosgraph_msgs.msg import Log

USE_GPU_ = 1

# Get the absolute path to the package
package_path = os.path.dirname(os.path.abspath(__file__))

# File for the values of the environment
filename_area = os.path.join(package_path, '..', 'config', 'filename_area.txt')
# Key to look or the specific environment
key = 'willowgarage'

# Retrieve the number of agents as parameter
# Modify the value before running
num_robots = rospy.get_param('~num_robots', 2)

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Callbacks~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Function to store the map discovered by all the agents together
def map_occupation(msg):
    global merged_x, merged_y
    area = map_area(msg)

    # Append the current time and area to the respective arrays
    merged_x.append(rospy.Time.now().to_sec())
    merged_y = np.append(merged_y, area)

# Callback to get the automatically generated map
def generated_map_callback(msg):
    global generated_map_area
    # Compute the total area of the environment
    generated_map_area = map_area(msg)

# Callback to get the total number of points passed to the server
def totalPointsCallback(data):
    global total_num_points, num_goals
    total_num_points.append(data.data)
    num_goals.append(len(total_num_points))

# Callback to get the actual number of points used for the reward computations
def actualPointsCallback(data):
    global actual_num_points
    actual_num_points.append(len(data.points))

def cpugpuCallback(msg):
    # Store also CPU and GPU usage
    monitor_cpu_usage()

    # Collect GPU data only if it is used
    if USE_GPU_:
        monitor_gpu_usage()

def radiusValueCallback(msg):
    # Store the radius value
    global radius_values
    radius_values.append(round(msg.data, 2))

def percentageValueCallback(msg):
    # Store the percentage value
    global percentage_values
    percentage_values.append(round(msg.data, 2))

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Functions~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Function to compute the total amount of area discovered
def map_area(map):
    discovered_cells = sum(1 for cell in map.data if (cell == 0 or cell == 100))
    return round(discovered_cells * (map.info.resolution ** 2), 2)


def monitor_cpu_usage():
    global cpu_time, cpu_percentages

    cpu_percent = psutil.cpu_percent(0.5)
    cpu_time.append(rospy.Time.now().to_sec())
    cpu_percentages = np.append(cpu_percentages, cpu_percent)


def monitor_gpu_usage():
    global gpu_time, gpu_percentages, gpu_memory

    gpu_list = GPUtil.getGPUs()
    for gpu in gpu_list:
        gpu_percent = float(gpu.load)*100
        gpu_mem = float(gpu.memoryUtil)*100
        gpu_time.append(rospy.Time.now().to_sec())
        gpu_percentages = np.append(gpu_percentages, gpu_percent)
        gpu_memory = np.append(gpu_memory, gpu_mem)


# Function to save the data for each simulation
def save_simulation_data(simulation_id):
    data = {
        'merged_x': merged_x,
        'merged_y': merged_y,
        'generated_map_area': generated_map_area,
        'cpu_time': cpu_time,
        'cpu_percentages': cpu_percentages,
        'gpu_time': gpu_time,
        'gpu_percentages': gpu_percentages,
        'gpu_memory': gpu_memory,
        'total_num_points': total_num_points,
        'actual_num_points': actual_num_points,
        'num_goals': num_goals,
        'radius_values': radius_values,
        'percentage_values': percentage_values
    }
    
    # Create the data directory if it doesn't exist
    data_directory = os.path.join(package_path, '..', 'data')
    if not os.path.exists(data_directory):
        os.makedirs(data_directory)
    
    filename = os.path.join(data_directory, f'data-{simulation_id}.npy')
    np.save(filename, data)

def get_area_value(filename, key):
    with open(filename, 'r') as file:
        for line in file:
            parts = line.strip().split(':')
            if len(parts) == 2:
                item = parts[0].strip()
                value = parts[1].strip()
                if item == key:
                    return float(value)
                
            
        # Key not found
        return 1

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Node~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
class PlotterNode(object):
    def __init__(self, simulation_id):
        global merged_x, merged_y, generated_map_area, cpu_percentages, gpu_percentages, gpu_memory, cpu_time, gpu_time, actual_num_points, total_num_points, num_goals, radius_values, percentage_values

        self.simulation_id = simulation_id

        rospy.init_node('plotter_node', anonymous=True)

        # Get initial time in seconds
        self.initial_time = rospy.Time.now().to_sec()

        # Subscriber for the merged map
        rospy.Subscriber('/map', OccupancyGrid, map_occupation)

        # Subscriber for the generated map
        rospy.Subscriber('/map2d', OccupancyGrid, generated_map_callback)

        # Subscriber to retrieve the list of the actual points used
        rospy.Subscriber("/merged_centroids", PointArray, actualPointsCallback)

        # Subscriber to retrieve the list of the total points used
        rospy.Subscriber("/list_points", Int32, totalPointsCallback)

        # Subscriber to retrieve the radius value used
        rospy.Subscriber("/radius_value", Float32, radiusValueCallback)

        # Subscriber to retrieve the radius value used
        rospy.Subscriber("/percentage_value", Float32, percentageValueCallback)

        # Subscriber to retrieve the CPU and GPU usage
        rospy.Subscriber("/rosout_agg", Log, cpugpuCallback)

        # Arrays to store x-values (time) and y-values (area)
        merged_x = []
        merged_y = np.array([])

        # Initialize generated_map_area
        generated_map_area = get_area_value(filename_area, key)

        # Arrays to store the values of usage for CPU ...
        cpu_time = []
        cpu_percentages = np.array([])
        # ... and GPU
        gpu_time = []
        gpu_percentages = np.array([])
        gpu_memory = np.array([])

        # Variables to store the number of goals, total and the actual number of points
        num_goals = []
        total_num_points = []
        actual_num_points = []

        # Variable to store the radius and percentage values used
        radius_values = []
        percentage_values = []

        rospy.on_shutdown(self.shutdown)

        # Spin the node to prevent it from exiting immediately
        rospy.spin()

    def shutdown(self):
        rospy.loginfo("Shutting down the plotter node...")

        # Save the data for the current simulation
        save_simulation_data(self.simulation_id)

        self.plot_figure()

        rospy.sleep(1)

    def plot_figure(self):
        global merged_x, merged_y, generated_map_area

        plt.figure(figsize=(16, 9))
        # Plot the data
        plt.plot(np.array(merged_x)-merged_x[0], (merged_y/generated_map_area)*100, label='Percentage of area discovered over time')

        plt.xlabel('Time [s]')
        plt.ylabel('Percentage map discovered')
        plt.legend()
        plt.title('Merged map occupation over time')
        plt.grid(True)

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


if __name__ == '__main__':
    # Create a unique ID for each simulation
    filename = os.path.join(package_path, '..', 'config', 'number_simulations.txt')
    with open(filename, 'r') as file:
        n = int(file.read())
    current_simulation_id = f"simulation_{n}_{num_robots}_robots"
    # Save the updated number back to the file
    with open(filename, 'w') as file:
        file.write(str(n + 1))

    node = PlotterNode(current_simulation_id)
