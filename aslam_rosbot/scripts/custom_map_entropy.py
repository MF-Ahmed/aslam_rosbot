#!/usr/bin/env python3
import numpy as np
import rospy
import math
import csv
from numpy.linalg import norm
from constants import csv_path_1, csv_path_2, csv_path_3
from PIL import Image, ImageDraw
import cv2
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid
from PIL import Image


def map_publisher():
    # Initialize the ROS node
    rospy.init_node('custom_map_entropy', anonymous=True)

    # Load the occupancy grid image
    occupancy_grid_image = cv2.imread('occupancy_grid.png', cv2.IMREAD_GRAYSCALE)
    occupancy_grid_image = cv2.resize(occupancy_grid_image, (50, 50))

    # Convert the occupancy grid image to a numpy array
    occupancy_grid = np.uint8((255 - occupancy_grid_image) * 100 / 255)

    occupancy_grid[10,10] = 90
    occupancy_grid[11,11] = 50
    occupancy_grid[12,12] = 10
    # Create the occupancy grid message
    occupancy_grid_msg = OccupancyGrid()
    occupancy_grid_msg.header.stamp = rospy.Time.now()
    occupancy_grid_msg.header.frame_id = 'new_map'
    occupancy_grid_msg.info.resolution = 0.05 # change the value based on your occupancy grid resolution
    occupancy_grid_msg.info.width = occupancy_grid.shape[1]
    occupancy_grid_msg.info.height = occupancy_grid.shape[0]
    occupancy_grid_msg.info.origin.position.x = -occupancy_grid.shape[1] * occupancy_grid_msg.info.resolution / 2
    occupancy_grid_msg.info.origin.position.y = -occupancy_grid.shape[0] * occupancy_grid_msg.info.resolution / 2
    occupancy_grid_msg.info.origin.orientation.w = 1
    occupancy_grid_msg.data = list(occupancy_grid.flatten())
    # Create the occupancy grid publisher
    occupancy_grid_pub = rospy.Publisher('new_map', OccupancyGrid, queue_size=10)

    # Publish the occupancy grid message repeatedly
    rate = rospy.Rate(10) # 10 Hz
    while not rospy.is_shutdown():
        occupancy_grid_msg.header.stamp = rospy.Time.now()
        occupancy_grid_pub.publish(occupancy_grid_msg)
        rate.sleep()

def compute_entropy(occupency_values):
    entropy=0
    try:
       for occupancy_value in occupency_values:
            if occupancy_value == -1:
                prob=0.100  # yeilds low entropy and high uncertinaty and low information gain
            elif occupancy_value == 0 or occupancy_value == 100:
                prob=0.450 #yields high entropy and low uncertinaty and high information gain
            else:
               print("I got a different occupancy value of {}".format(occupancy_value))
            try:
                entropy +=  (-((prob * math.log2(prob) + (1 - prob)*math.log2(1 - prob)))) #* np.exp(-0.25 *  self.euclidean_distance(robotposx,robotposy, frontierposx, frontierposy))

            except Exception as e:
                print("problem computing the entropy {}".format(e))
        # Do something with the occupancy value
    except Exception as e:
        print("Error processing compute entropy method: {}".format(e))

    #if entropy !=0 and len(occupency_values)!=0:
        #entropy = entropy/len(occupency_values)

    return entropy/len(occupency_values)

def savetofile(self,data,frontierID):
    try:
        with open(csv_path_1, mode='w') as file:
            writer = csv.writer(file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            rr,cc = np.shape(data)
            writer.writerow("Frontier No. "+str(frontierID))
            for i in range (0,cc):
                pose = [str(data[0,i]),str(data[1,i])]
                writer.writerow(pose)

    except IOError as e:
        print("Unable to write CSV file: %s", str(e))

    finally:
        file.close()

def euclidean_distance(self, x1, y1, x2, y2):
    distance = math.sqrt(math.pow(x2 - x1, 2) + math.pow(y2 - y1, 2))
    return distance

if __name__ == '__main__':
    try:
        occupency_values = [0, 0, 0, 0, 0, -1, -1, -1]
        entropy = compute_entropy(occupency_values)
        print("entropy is = {}".format(entropy))
        map_publisher()

    except rospy.ROSInterruptException:
        pass

    







