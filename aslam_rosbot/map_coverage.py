import cv2
import numpy as np

def compute_coverage_percentage(larger_map_file, smaller_map_file):
    # Load the larger and smaller map images
    larger_map = cv2.imread(larger_map_file, 0)
    smaller_map = cv2.imread(smaller_map_file, 0)

    # Resize the smaller map to match the dimensions of the larger map
    smaller_map_resized = cv2.resize(smaller_map, (larger_map.shape[1], larger_map.shape[0]))

    # Threshold the images to convert them to binary images
    _, larger_map_thresholded = cv2.threshold(larger_map, 210, 255, cv2.THRESH_BINARY)
    _, smaller_map_thresholded = cv2.threshold(smaller_map_resized, 210, 255, cv2.THRESH_BINARY)

    # Compute the intersection of the two images
    intersection = cv2.bitwise_and(larger_map_thresholded, smaller_map_thresholded)

    # Calculate the coverage percentage
    coverage_percentage = np.sum(intersection) / np.sum(smaller_map_thresholded) * 100

    return coverage_percentage

# Example usage
larger_map_file = "maps_fortets_L.pgm"
smaller_map_file = "maps_fortets_S.pgm"
percentage_coverage = compute_coverage_percentage(larger_map_file, smaller_map_file)
print("Percentage Coverage:", percentage_coverage, "%")