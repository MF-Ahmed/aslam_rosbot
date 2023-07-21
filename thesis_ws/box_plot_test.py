import matplotlib.pyplot as plt
import numpy as np

# Sample data for two datasets
data1 = np.random.normal(loc=0, scale=1, size=100)
data2 = np.random.normal(loc=2, scale=1, size=100)
data3 = np.random.normal(loc=0, scale=1, size=100)
data4 = np.random.normal(loc=2, scale=1, size=100)

# Combine the data into a list of lists
data = [data1, data2,data3, data4]

# Create a figure and axis
fig, ax = plt.subplots()

# Plot two sets of data on a box plot graph
bp = ax.boxplot(data, patch_artist=True)

# Set labels for the boxes
labels = ['Dataset 1', 'Dataset 2','Dataset 3', 'Dataset 4']
ax.set_xticklabels(labels)

# Set labels for x and y axes, and the title
ax.set_xlabel('Datasets')
ax.set_ylabel('Values')
ax.set_title('Box Plot with Value Annotation')

# Customize the boxplot colors
box_colors = ['skyblue', 'lightgreen']
for box, color in zip(bp['boxes'], box_colors):
    box.set(facecolor=color)

# Add vertical lines at specific x-axis positions
vertical_lines_x = [1.5]  # Add vertical line at x=1.5 (between the two datasets)
for x in vertical_lines_x:
    plt.axvline(x, color='red', linestyle='dashed', linewidth=2)

# Display a value on the box plot with an offset on the x-axis
value_to_display = 2.5
offset_x = 0.15
ax.text(value_to_display + offset_x, 0.5, f'{value_to_display}', color='red', fontsize=12)

plt.show()