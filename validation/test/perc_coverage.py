import numpy as np
from PIL import Image

def compute_coverage(file1, file2):
    # Read the PGM files using PIL
    img1 = Image.open(file1)
    img2 = Image.open(file2)

    # Resize the images to match dimensions
    img2 = img2.resize(img1.size)

    # Convert the images to NumPy arrays
    arr1 = np.array(img1)
    arr2 = np.array(img2)

    # Calculate the total number of pixels
    total_pixels = arr1.size

    # Count the number of matching pixels
    matching_pixels = np.sum(arr1 == arr2)

    # Calculate the percentage coverage
    coverage = (matching_pixels / total_pixels) * 100

    return coverage

# Usage example
file1 = "/home/usr/data/catkin_ws/src/aslam_turtlebot/aslam_turtlebot_27thJune_willow_1/full_willow_map.pgm"  # orignal map file
#file2 = "/home/usr/data/catkin_ws/src/aslam_turtlebot/aslam_turtlebot_27thJune_office_1/office_full_clean.pgm"
file2 = "/home/usr/data/catkin_ws/src/aslam_rosbot/aslam_rosbot_27thJune_our_willow_2/_mpc_rosbot_27thJune.pgm"  #
coverage = compute_coverage(file1, file2)
print("Percentage Coverage:", coverage)