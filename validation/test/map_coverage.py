import numpy as np
from PIL import Image

def read_pgm(file_path):
    with open(file_path, 'rb') as f:
        # Read the PGM header
        header = f.readline().decode(errors='replace').strip()
        if header != 'P5':
            raise ValueError('Invalid PGM file format')

        # Skip any comments
        while True:
            line = f.readline().decode(errors='replace').strip()
            if not line.startswith('#'):
                break

        # Read the width, height, and maximum gray value
        width, height = map(int, line.split())
        max_gray_value = int(f.readline().decode(errors='replace').strip())

        # Read the pixel values
        data = np.zeros((height, width), dtype=np.uint8)
        for y in range(height):
            for x in range(width):
                try:
                    value = int(f.readline().decode(errors='replace').strip())
                except ValueError:
                    continue  # Skip line if conversion fails
                data[y, x] = value

        return data

def resize_map(map_data, target_width, target_height):
    img = Image.fromarray(map_data)
    resized_img = img.resize((target_width, target_height), resample=Image.NEAREST)
    resized_map_data = np.array(resized_img)
    return resized_map_data

def compute_coverage(map1, map2, threshold):
    # Step 1: Read the map files
    map1_data = read_pgm(map1)
    map2_data = read_pgm(map2)

    # Step 2: Extract the dimensions
    map1_width, map1_height = map1_data.shape
    map2_width, map2_height = map2_data.shape

    # Step 3: Normalize pixel values if necessary

    # Step 4: Resize maps to match dimensions
    if map1_width != map2_width or map1_height != map2_height:
        max_width = max(map1_width, map2_width)
        max_height = max(map1_height, map2_height)

        map1_data = resize_map(map1_data, max_width, max_height)
        map2_data = resize_map(map2_data, max_width, max_height)

    # Step 5: Compute coverage
    total_covered_pixels = 0
    total_pixels = map1_data.size
    for pixel_value_map1, pixel_value_map2 in zip(map1_data.flatten(), map2_data.flatten()):
        if pixel_value_map1 > threshold and pixel_value_map2 > threshold:
            total_covered_pixels += 1

    # Step 6: Compute coverage percentage
    coverage_percentage = (total_covered_pixels / total_pixels) * 100
    return coverage_percentage

# Example usage
map1 = "/home/usr/data/catkin_ws/src/aslam_rosbot/aslam_rosbot_27thJune_julio_office_1/office_full.pgm"
map2 = "/home/usr/data/catkin_ws/src/aslam_rosbot/aslam_rosbot_27thJune_julio_office_1/office_full.pgm"
threshold = -10

coverage = compute_coverage(map1, map2, threshold)
print(f"Coverage percentage: {coverage:.2f}%")
