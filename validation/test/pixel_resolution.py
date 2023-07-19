
from PIL import Image
def get_pgm_resolution(file_path):
    with open(file_path, 'rb') as file:
        # Read the first line (magic number) and ignore it
        file.readline()
        # Read the second line (comment) and ignore it
        file.readline()
        # Read the third line (width and height)
        width, height = map(int, file.readline().decode().split())
        return width, height

def compute_map_size(pgm_file_path, pixel_resolution_x, pixel_resolution_y):
    width, height = get_pgm_resolution(pgm_file_path)
    size_x = width * pixel_resolution_x
    size_y = height * pixel_resolution_y
    return size_x, size_y

# Example usage
#pgm_file_path = "/home/usr/data/catkin_ws/src/aslam_rosbot/aslam_rosbot_27thJune_julio_office_1/office_full_clean.pgm"
pgm_file_path = "/home/usr/data/catkin_ws/src/aslam_rosbot/aslam_rosbot_27thJune_Julio_willow_1/full_willow_map.pgm"

#pgm_file_path = "/home/usr/data/catkin_ws/src/aslam_rosbot/aslam_rosbot_27thJune_julio_office_1/office_full.pgm"
pixel_resolution_x = 0.05  # Assuming the pixel resolution in the X direction is 0.1 meters/pixel
pixel_resolution_y = 0.05  # Assuming the pixel resolution in the Y direction is 0.1 meters/pixel
map_size = compute_map_size(pgm_file_path, pixel_resolution_x, pixel_resolution_y)
print("Map Size (X, Y):", map_size)
