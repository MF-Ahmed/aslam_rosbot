def plot_cells_on_map(ray_cells):
    """ Plot the cells on the occupancy grid map """
    global map_data
    for cell in ray_cells:
        if cell[0] >= 0 and cell[0] < map_data.shape[1] and cell[1] >= 0 and cell[1] < map_data.shape[0]:
            map_data[cell[1], cell[0]] = 100

if __name__ == '__main__':
    rospy.init_node('ray_tracing')
    rospy.Subscriber('/map', OccupancyGrid, get_map_data)
    rospy.sleep(1)  # Wait for the occupancy grid map data to be received
    start_point = Point(0, 0, 0)
    end_point = Point(1, 1, 0)
    ray_cells = ray_tracing(start_point, end_point)
    plot_cells_on_map(ray_cells)
    map_msg = OccupancyGrid()
    map_msg.header.stamp = rospy.Time.now()
    map_msg.header.frame_id = 'map'
    map_msg.info.resolution = map_resolution
    map_msg.info.width = map_data.shape[1]
    map_msg.info.height = map_data.shape[0]
    map_msg.info.origin.position = Point(map_origin[0], map_origin[1], 0)
    map_msg.info.origin.orientation = Quaternion(0, 0, 0, 1)
    map_msg.data = map_data.reshape(-1).tolist()
    map_pub = rospy.Publisher('/map_ray_traced', OccupancyGrid, queue_size=1)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        map_msg.header.stamp = rospy.Time.now()
        map_pub.publish(map_msg)
        rate.sleep()
