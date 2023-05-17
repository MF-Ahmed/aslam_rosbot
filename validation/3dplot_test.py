import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Create data points
x = np.linspace(-5, 5, 10)
y = np.linspace(-5, 5, 10)

x, y = np.meshgrid(x, y)
z = np.sin(np.sqrt(x**2 + y**2))

print("x = {}".format(x.shape))
print("y = {}".format(y.shape))
print("z = {}".format(z.shape))

# Create figure and 3D axes
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the surface
ax.plot_surface(x, y, z, cmap='viridis')

# Set labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('3D Plot')

# Show the plot
plt.show()

#This code generates a 3D plot of the surface defined by the equation z = sin(sqrt(x^2 + y^2)). It uses numpy to create the data points, matplotlib to create the plot, and mpl_toolkits.mplot3d to enable 3D plotting.

#Make sure you have the necessary libraries installed (numpy, matplotlib), and then you can run this code to see the 3D plot. Feel free to modify the data points or the equation to suit your needs.
