import numpy as np
import matplotlib.pyplot as plt

def weightage(distance, decay_factor):
    return np.exp(-decay_factor * distance)

decay_factor = 0.1  # Adjust this value to control the decay rate
distances = np.linspace(0, 20, 100)  # Generate an array of distances from 0 to 20 meters

weights = weightage(distances, decay_factor)

# Plotting
plt.plot(distances, weights)
plt.xlabel('Distance (m)')
plt.ylabel('Weightage')
plt.title('Weightage as a function of distance')
plt.grid(True)
plt.show()