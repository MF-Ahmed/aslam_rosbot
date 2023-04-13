import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

def get_map_data(map_msg):
    """ Callback function for receiving the occupancy grid map data """
    global map_data, map_resolution, map_origin
    map_data = np.array(map_msg.data, dtype=np.int8).reshape(map_msg.info.height, map_msg.info.width)
    map_resolution = map_msg.info.resolution
    map_origin = (map_msg.info.origin.position.x, map_msg.info.origin.position.y)

def get_map_cell(point):
    """ Convert a point in the world coordinates to a cell in the occupancy grid map """
    cell_x = int((point.x - map_origin[0]) / map_resolution)
    cell_y = int((point.y - map_origin[1]) / map_resolution)
    return (cell_x, cell_y)

def ray_tracing(start_point, end_point):
    """ Perform ray tracing from start_point to end_point """
    start_cell = get_map_cell(start_point)
    end_cell = get_map_cell(end_point)
    ray_cells = []
    x0, y0 = start_cell
    x1, y1 = end_cell
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x = x0
    y = y0
    n = 1 + dx + dy
    x_inc = 1 if x1 > x0 else -1
    y_inc = 1 if y1 > y0 else -1
    error = dx - dy
    dx *= 2
    dy *= 2
    while n > 0:
        ray_cells.append((x, y))
        if error >= 0:
            x += x_inc
            error -= dy
        else:
            y += y_inc
            error += dx
        n -= 1
    return ray_cells

if __name__ == '__main__':
    rospy.init_node('ray_tracing')
    rospy.Subscriber('/map', OccupancyGrid, get_map_data)
    rospy.sleep(1)  # Wait for the occupancy grid map data to be received
    start_point = Point(0, 0, 0)
    end_point = Point(1, 1, 0)
    ray_cells = ray_tracing(start_point, end_point)
    print('Ray cells:', ray_cells)