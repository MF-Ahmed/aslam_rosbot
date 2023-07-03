from PIL import Image

# Load the .pgm map file
image = Image.open("maps_fortets.pgm")
# Get the dimensions of the map in pixels
width, height = image.size

# Define the resolution in meters/pixel
resolution_meters_per_pixel = 0.05

# Compute the area in square meters
area_square_meters = (width * resolution_meters_per_pixel) * (height * resolution_meters_per_pixel)
print("Width of the map:", width * resolution_meters_per_pixel, "meters")
print("Height of the map:", height  * resolution_meters_per_pixel, "meters")
print("Area of the map:", area_square_meters, "square meters")