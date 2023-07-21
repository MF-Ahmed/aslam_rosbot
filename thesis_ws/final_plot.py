import numpy as np
import matplotlib.pyplot as plt
import os
import seaborn as sns

# Define the directory where the data files are located
file_path = os.path.dirname(os.path.abspath(__file__))
data_directory = os.path.join(file_path, 'src', 'plotting', 'data')

# Get a list of all the data files in the directory and sort them based on the simulation number
data_files = sorted([file for file in os.listdir(data_directory) if file.startswith('data-simulation')], key=lambda x: int(x.split("_")[1]))

# Initialize lists to store the data from each simulation
merged_x_list = []
merged_y_list = []
generated_map_area_list = []
cpu_time_list = []
cpu_percentages_list = []
gpu_time_list = []
gpu_percentages_list = []
gpu_memory_list = []
num_robots_list = []
num_goals_list = []
total_num_points_list = []
actual_num_points_list = []
radius_values_list = []
percentage_values_list = []
num_simulations = 0

# Iterate over the data files
for file in data_files:

    num_simulations += 1
    # Load the data from the file
    data = np.load(os.path.join(data_directory, file), allow_pickle=True).item()

    # Extract the required data from the simulation
    merged_x = data['merged_x']
    merged_y = data['merged_y']
    generated_map_area = data['generated_map_area']
    cpu_time = data['cpu_time']
    cpu_percentages = data['cpu_percentages']
    gpu_time = data['gpu_time']
    gpu_percentages = data['gpu_percentages']
    gpu_memory = data['gpu_memory']
    num_goals = data['num_goals']
    total_num_points = data['total_num_points']
    actual_num_points = data['actual_num_points']
    radius_values = data['radius_values']
    percentage_values = data['percentage_values']

    # Extract the number of robots from the file name
    num_robots = int(file.split('_')[2].split('.')[0])
    num_robots_list.append(num_robots)

    # Append the data to the respective lists
    merged_x_list.append(merged_x)
    merged_y_list.append(merged_y)
    generated_map_area_list.append(generated_map_area)
    cpu_time_list.append(cpu_time)
    cpu_percentages_list.append(cpu_percentages)
    gpu_time_list.append(gpu_time)
    gpu_percentages_list.append(gpu_percentages)
    gpu_memory_list.append(gpu_memory)
    num_goals_list.append(num_goals)
    total_num_points_list.append(total_num_points)
    actual_num_points_list.append(actual_num_points)
    radius_values_list.append(radius_values)
    percentage_values_list.append(percentage_values)

################################## MAP OCCUPATION ##################################
# Plotting the data for the map occupancy
''''
plt.figure(figsize=(16, 9))

# Plot the merged map occupation over time for each simulation
for i in range(num_simulations):
    _simulation_number = int(data_files[i].split("_")[1])
    plt.plot(np.array(merged_x_list[i]) - merged_x_list[i][0], (merged_y_list[i] / generated_map_area_list[i]) * 100,
             label=f'Simulation {_simulation_number} - {num_robots_list[i]} Robots',linewidth=3)

plt.xlabel('Time [s]')
plt.ylabel('Percentage map discovered')
plt.legend()
plt.title('Merged map occupation over time')
plt.grid(True)

# Since the plot concern a percentage the graph will be from 0 to 100
plt.ylim(0, 100)
#plt.xlim(0, 1200)

# Adjust the figure
plt.subplots_adjust(left=0.05, bottom=0.05, right=0.95, top=0.95, wspace=0.2, hspace=0.4)

# Show the plot
plt.show()

################################## NUM POINTS USED ##################################

plt.figure(figsize=(16, 9))

# Lists to store data for box plots
total_points_data = []
actual_points_data = []
num_robots_list = []

# Process data to prepare for box plots
for i, (_, total_points) in enumerate(zip(num_goals_list, total_num_points_list)):
    # Append data to the corresponding lists for box plots
    total_points_data.append(total_points)
    actual_points_data.append(actual_num_points_list[i])

    # Extract the number of robots from the file name
    num_robots = int(data_files[i].split("_")[2])
    num_robots_list.append(num_robots)

# Create box plots with consistent styles for each simulation

boxprops = dict(linewidth=2, color='black')  # Style for the box outline
whiskerprops = dict(linewidth=2, color='black')  # Style for the whiskers
medianprops = dict(linewidth=2, color='red')  # Style for the median line

# Calculate the x-axis positions for the boxes
x_positions_total = np.arange(1, len(total_points_data) * 4 + 1, 4)
x_positions_actual = np.arange(2, len(actual_points_data) * 4 + 2, 4)

# Create the box plots and get the artists
total_boxes = plt.boxplot(total_points_data, positions=x_positions_total,
                         patch_artist=True, boxprops=boxprops, whiskerprops=whiskerprops, medianprops=medianprops)

actual_boxes = plt.boxplot(actual_points_data, positions=x_positions_actual,
                          patch_artist=True, boxprops=boxprops, whiskerprops=whiskerprops, medianprops=medianprops)

# Set the colors of the box plots
for box in total_boxes['boxes']:
    box.set_facecolor('lightblue')
for box in actual_boxes['boxes']:
    box.set_facecolor('lightgreen')

# Set the labels for the x-axis based on the simulation number and number of robots
x_labels = []
for i, num_robots in enumerate(num_robots_list):
    simulation_label = f'Simulation {int(data_files[i].split("_")[1])}'
    x_labels.append(f'{simulation_label}\n({num_robots} Robots)')

plt.xticks(np.mean([x_positions_total, x_positions_actual], axis=0), x_labels)

# Create custom artists for the legend
total_patch = plt.Line2D([], [], color='lightblue', marker='o', markersize=10, label='Total Points Detected')
actual_patch = plt.Line2D([], [], color='lightgreen', marker='o', markersize=10, label='Actual Points Used')

# Set the legend with the custom artists and mean line
plt.legend(handles=[total_patch, actual_patch])

plt.xlabel('Simulations')
plt.ylabel('Number of points')
plt.title('Box plots for Total and Actual number of points used')
plt.grid(True)

# Show the plot
plt.show()
'''

################################## RADIUS VALUES USED ##################################
# Plotting the data for the map occupancy

plt.figure(figsize=(16, 9))

#plt.hlines(1.0, xmin=num_goals[0], xmax=num_goals[-1], color='r', label='First value of radius')

for i, (goals, radius_values) in enumerate(zip(num_goals_list, radius_values_list)):
    _simulation_number = int(data_files[i].split("_")[1])

    #plt.scatter(goals, radius_values, marker='x', s=50,
                   #label=f'Simulation {_simulation_number} - Radius Values')
    plt.plot(goals, radius_values)

plt.xlabel('Number of goals')
plt.ylabel('Radius Values')
plt.legend()
plt.title('Radius Values and number of goals Used')
plt.grid(True)

# Min and max for plot
#min_y = np.min([np.min(y) for y in radius_values])  # Minimum value over all simulations
#max_y = np.max([np.max(y) for y in radius_values])  # Maximum value over all simulations

#plt.ylim(min_y - 0.25, max_y + 0.25)

# Adjust the figure
#plt.subplots_adjust(left=0.05, bottom=0.05, right=0.95, top=0.95, wspace=0.2, hspace=0.4)

# Show the plot
plt.show()

################################## PERCENTAGE VALUES USED ##################################

# Plotting the data for the map occupancy
plt.figure(figsize=(16, 9))

plt.hlines(60.0, xmin=num_goals[0], xmax=num_goals[-1], color='r', label='First value of radius')
for i, (goals, percentage_values) in enumerate(zip(num_goals_list, percentage_values_list)):
    _simulation_number = int(data_files[i].split("_")[1])
    plt.scatter(goals, percentage_values, marker='x', s=50,
                   label=f'Simulation {_simulation_number} - Percentage Values')

plt.xlabel('Number of goals')
plt.ylabel('Percentage Values')
plt.legend()
plt.title('Percentage Values Used')
plt.grid(True)

# Min and max for plot
min_y = np.min([np.min(y) for y in percentage_values])  # Minimum value over all simulations
max_y = np.max([np.max(y) for y in percentage_values])  # Maximum value over all simulations

plt.ylim(min_y - 0.5, max_y + 0.5)

# Adjust the figure
plt.subplots_adjust(left=0.05, bottom=0.05, right=0.95, top=0.95, wspace=0.2, hspace=0.4)

# Show the plot
plt.show()

################################## CPU USAGE ##################################
# Plotting the data for the map occupancy
'''

plt.figure(figsize=(16, 9))

# Plot the merged map occupation over time for each simulation
for i in range(num_simulations):
    _simulation_number = int(data_files[i].split("_")[1])
    plt.plot(np.array(cpu_time_list[i]) - cpu_time_list[i][0], cpu_percentages_list[i],
             label=f'Simulation {_simulation_number} - CPU Usage over time with {num_robots_list[i]} Robots')

plt.xlabel('Time [s]')
plt.ylabel('Percentage CPU used')
plt.legend()
plt.title('CPU Usage')
plt.grid(True)

plt.ylim(0, 100)

# Adjust the figure
plt.subplots_adjust(left=0.05, bottom=0.05, right=0.95, top=0.95, wspace=0.2, hspace=0.4)

# Show the plot
plt.show()

################################## GPU USAGE ##################################

if gpu_time != []:
    # Plotting the data for the map occupancy
    plt.figure(figsize=(16, 9))

    # Plot the merged map occupation over time for each simulation
    for i in range(num_simulations):
        _simulation_number = int(data_files[i].split("_")[1])
        plt.plot(np.array(gpu_time_list[i]) - gpu_time_list[i][0], gpu_percentages_list[i],
                label=f'Simulation {_simulation_number} - GPU Usage over time with {num_robots_list[i]} Robots')
        plt.plot(np.array(gpu_time_list[i]) - gpu_time_list[i][0], gpu_memory_list[i],
                label=f'Simulation {_simulation_number} - GPU Memory Usage over time with {num_robots_list[i]} Robots')
        
    plt.xlabel('Time [s]')
    plt.ylabel('Percentage GPU used')
    plt.legend()
    plt.title('GPU Usage')
    plt.grid(True)

    plt.ylim(0, 100)

    # Adjust the figure
    plt.subplots_adjust(left=0.05, bottom=0.05, right=0.95, top=0.95, wspace=0.2, hspace=0.4)

    # Show the plot
    plt.show()
'''