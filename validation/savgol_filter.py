import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter

# Generate noisy data
x = np.linspace(0, 10, 100)
print("x = {}".format(type(x[10])))
y_noisy = np.sin(x) + np.random.normal(0, 0.2, size=len(x))

# Apply Savitzky-Golay filter
window_size = 11
order = 2
y_filtered = savgol_filter(y_noisy, window_size, order)

# Plot original and filtered data
plt.plot(x, y_noisy, label='Noisy Data')
plt.plot(x, y_filtered, label='Filtered Data')
plt.legend()
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Noisy Data vs Savitzky-Golay Filtered Data')
plt.show()