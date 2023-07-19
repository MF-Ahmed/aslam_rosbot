import numpy as np
from PIL import Image

def compute_rmse(file1, file2):
    # Read the PGM files using PIL
    img1 = Image.open(file1)
    img2 = Image.open(file2)

    # Resize the images to match dimensions
    img2 = img2.resize(img1.size)

    # Convert the images to NumPy arrays
    arr1 = np.array(img1)
    arr2 = np.array(img2)

    # Compute the squared differences
    diff = (arr1 - arr2) ** 2

    # Calculate the mean squared error
    mse = np.mean(diff)

    # Compute the root mean square error
    rmse = np.sqrt(mse)

    return rmse

# Usage example

file1 = "/home/usr/data/catkin_ws/src/aslam_turtlebot/aslam_turtlebot_27thJune_office_1/office_full_clean.pgm"  # orignal map file
file2 = "/home/usr/data/catkin_ws/src/aslam_rosbot/aslam_rosbot_27thJune_julio_office_1/_rosbot_27thJune.pgm"  #
error = compute_rmse(file1, file2)
print("RMSE:", error)