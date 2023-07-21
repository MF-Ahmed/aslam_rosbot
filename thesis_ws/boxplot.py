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
total_patch = plt.Line2D([], [], color='lightblue', marker='o', markersize=10, label='Total Number of Points Detected')
actual_patch = plt.Line2D([], [], color='lightgreen', marker='o', markersize=10, label='Actual Number of Points Used')

# Set the legend with the custom artists and mean line
plt.legend(handles=[total_patch, actual_patch])

plt.xlabel('Simulations')
plt.ylabel('Number of points')
plt.title('Box plots for Total and Actual number of points used')
plt.grid(True)

# Show the plot
plt.show()