import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Generate some example data
x = np.linspace(0, 10, 100)
y = np.sin(x)

# Create a pandas DataFrame
data = pd.DataFrame({'x': x, 'y': y})

# Calculate the running average
window_size = 5
data['running_average'] = data['y'].rolling(window=window_size, min_periods=1).mean()

# Plot the original data
plt.plot(data['x'], data['y'], label='Original Data')

# Plot the running average
plt.plot(data['x'], data['running_average'], label='Running Average')

# Add labels and a legend
plt.xlabel('X')
plt.ylabel('Y')
plt.legend()

# Display the graph
plt.show()