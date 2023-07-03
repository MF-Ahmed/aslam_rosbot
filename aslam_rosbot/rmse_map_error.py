import cv2
import numpy as np

def compute_rmse(map_file1, map_file2):
    # Load the map images
    map1 = cv2.imread(map_file1, 0)
    map2 = cv2.imread(map_file2, 0)

    # Ensure the dimensions of the maps are the same
    assert map1.shape == map2.shape, "Map dimensions do not match."

    length= map1.shape[0] * 0.05
    width = map1.shape[1] * 0.05
    # Compute the squared differences
    squared_diff = (map1.astype(np.float64) - map2.astype(np.float64)) ** 2

    # Compute the mean squared difference
    mse = np.mean(squared_diff)

    # Compute the RMSE
    rmse = np.sqrt(mse)/(length*width)

    return rmse

# Example usage
map_file1 = "1.pgm"
map_file2 = "2.pgm"
rmse = compute_rmse(map_file1, map_file2)
print("RMSE:", rmse)
