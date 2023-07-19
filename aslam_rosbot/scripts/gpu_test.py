import numpy as np
from numba import cuda, jit
import psutil
import time

# Define a GPU-accelerated function
@cuda.jit
def add_arrays_gpu(a, b, c):
    i = cuda.grid(1)
    if i < c.size:
        c[i] = a[i] + b[i]

# Function to get CPU usage percentage
def get_cpu_usage():
    return psutil.cpu_percent(interval=1)

# Create input arrays
a = np.ones(100)
b = np.ones(100)
c = np.zeros_like(a)

# Allocate device memory
d_a = cuda.to_device(a)
d_b = cuda.to_device(b)
d_c = cuda.to_device(c)

# Configure the grid and block dimensions
block_dim = 32
grid_dim = (a.size + block_dim - 1) // block_dim

# Print CPU usage every second
while True:
    # Start time for CPU usage monitoring
    start_time = time.time()

    # Launch the GPU kernel
    add_arrays_gpu[grid_dim, block_dim](d_a, d_b, d_c)

    # Copy the result back to the host
    d_c.copy_to_host(c)

    # Print the result
    print(c)

    # Calculate elapsed time for CPU usage monitoring
    elapsed_time = time.time() - start_time

    # Print CPU usage
    cpu_usage = get_cpu_usage()
    print(f"CPU Usage: {cpu_usage}%")

    # Pause for the remaining time in the second
    time.sleep(max(1 - elapsed_time, 0))
