#!/usr/bin/env python3
import rospy
import numpy as np
import roslib
import math
import csv
from numpy.linalg import norm
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped, Pose, Point
from aslam_rosbot.msg import PointArray
from aslam_rosbot.msg import FrontierWithPath
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from nav_msgs.msg import Path, OccupancyGrid
from functions import *
from constants import csv_path_1, csv_path_2, csv_path_3
from PIL import Image, ImageDraw
#from occupancy_grid_utils.ray_trace import bresenham2D

from bresenham import bresenham

from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Polygon
from tf import transformations # rotation_matrix(), concatenate_matrices()


from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import ColorRGBA


class ComputeEntropy(object):
    def __init__(self):
        rospy.Subscriber('/filtered_points', PointArray, self.callback)        
        rospy.Subscriber("/map", OccupancyGrid, self.mapcallback)        
        rospy.Subscriber('frontier_path', FrontierWithPath, self.frontiercallback)
        rospy.Subscriber('chosen_frontier', FrontierWithPath, self.chosenfrontiercallback)
        self.frontier_infongain = rospy.Publisher("frontier_onfigain", FrontierWithPath, queue_size=10)


        self.marker_pub = rospy.Publisher("visualization_marker/EntropyPath", Marker, queue_size=10)
        self.markerArray_pub = rospy.Publisher("visualization_markerArray/EntropyPath2", MarkerArray, queue_size=10)
        self.frontier_plan= FrontierWithPath()
        self.occupancy_value= 0.0
        
        self.PoseArray=[]
        #self.map_msg = OccupancyGrid()
        self.path_pose =[]
     
    def draw_marker(self,x, y, color=[1.0,0.0,0.0], mtype="sphere", scale=0.1, ns='my_marker_ns'):
        # Create a Marker message
        #rospy.loginfo("got x at {} and y at {}".format(x,y))
        marker = Marker()
        marker.header.frame_id = "map"       
        marker.header.stamp = rospy.Time.now() 
        marker.action = marker.ADD
        #marker.id = 0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.a = 1.0
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.lifetime = rospy.Duration(1.5)
        if mtype == "point":
            marker.type = Marker.POINTS
            marker.scale.x = marker.scale.y = scale
        elif mtype == "sphere":
            marker.type = Marker.SPHERE
            marker.scale.x = marker.scale.y = marker.scale.z = scale  # Diameter
        elif mtype == "arrow":
            marker.type = Marker.ARROW
            marker.scale.x = scale  # Arrow length
            marker.scale.y = marker.scale.z = 0.05  # Arrow head diameter and length
        elif mtype == "cube":
            marker.type = Marker.CUBE
            marker.scale.x = marker.scale.y = marker.scale.z = scale
        elif mtype == "circumference":
            marker.type = Marker.SPHERE
            marker.scale.x = marker.scale.y = scale
            marker.scale.z = 0.05
            marker.pose.position.z = 0.0
        elif mtype == "lines":
            marker.type = Marker.LINE_STRIP
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.0
            marker.scale.x = scale
        return marker
        #rate = rospy.Rate(10) # 10Hz        
        #while not rospy.is_shutdown():     
        #    rate.sleep()

    def chosenfrontiercallback(self, data):

        self.chosenfrontier_posx = data.chosenfrontierxy_pos.x   
        self.chosenfrontier_posy = data.chosenfrontierxy_pos.y  

        markerArray = MarkerArray()              
        marker = Marker()
        #value=gridValue(self.map_msg, Xp)        
        #rospy.loginfo("------- occupancy_value  at Xp is  {} ----------------".format(value))          
        marker = self.draw_marker(data.chosenfrontierxy_pos.x,data.chosenfrontierxy_pos.y, [0.5,0.1,0.7],"cube",0.8)
        marker.id= 200
        self.marker_pub.publish(marker) 

    def path_planner_callback(self, data):   
        pass

    def mapcallback(self,mapdata):
        self.map_msg = mapdata

        global map_data, map_resolution, map_origin, map_width, map_height
        map_data = np.array(self.map_msg.data, dtype=np.int8).reshape(self.map_msg.info.height, self.map_msg.info.width)
        map_resolution = self.map_msg.info.resolution
        map_origin = (self.map_msg.info.origin.position.x, self.map_msg.info.origin.position.y)
        map_width = mapdata.info.width
        map_height = mapdata.info.height  

    def get_map_cell(self,point):
        """ Convert a point in the world coordinates to a cell in the occupancy grid map """       
        cell_x = int((point.x - self.map_msg.info.origin.position.x) / self.map_msg.info.resolution)
        cell_y = int((point.y - self.map_msg.info.origin.position.y) / self.map_msg.info.resolution)
        return (cell_x, cell_y)

    def ray_tracing(self,start_point, end_point):
        """ Perform ray tracing from start_point to end_point """
        start_cell = self.get_map_cell(start_point)
        end_cell = self.get_map_cell(end_point)

        distance = math.sqrt(math.pow(end_cell[0] - start_cell[0], 2) + math.pow(end_cell[1] - start_cell[1], 2))  

        ray_cells = []
        occupancy_values=[]
        
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
        #if i >= 0 and i < self.map_msg.info.width and j >= 0 and j < self.map_msg.info.height:         
        #rospy.loginfo("map_data = {}".format(len(map_data)))
        #rospy.loginfo("ray_cells = {}".format(len(ray_cells)))

        #try:   
            #for cell in ray_cells:
                #if cell[0] >= 0 and cell[0] < self.map_msg.info.width and cell[1] >= 0 and cell[1] < self.map_msg.info.height:                           
                    #occupancy_values.append([map_data[cell[0], cell[1]]]) #= [map_data[cell[0], cell[1]]]
        #except Exception as e:  
            #rospy.logerr("problem computing the occupancy {}".format(e))  



        return  distance, ray_cells
        
    def getrobotposition(self):
        self.listener = tf.TransformListener()   
        self.listener.waitForTransform('map', 'base_link', rospy.Time(0), rospy.Duration(1.0))
        cond = 0
        while cond == 0:
            try:
                rospy.loginfo('..... waiting for the robot transform....')
                (trans, rot) = self.listener.lookupTransform('map', 'base_link', rospy.Time(0))
                robot_position = np.array([trans[0], trans[1]])
                cond = 1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                # rospy.logerr(tf.LookupException);
                cond = 0
               

        return  robot_position       


    def compute_path_entropy(self,frontier_plan):  

        self.markerArray = MarkerArray()       
        #rospy.loginfo("----------Computing the entropy for the frontier = {}  ----------------".format(frontier_plan.ID)) 
        #rospy.loginfo("----------Spanning trees for the frontier = {}  ----------------".format(frontier_plan.spanning_trees)) 
                
        robot_position = self.getrobotposition()
        entropy=0        

        start_point = Point( robot_position[0] ,  robot_position[1] , 0)
        end_point = Point( frontier_plan.frontier_loc.x , frontier_plan.frontier_loc.y, 0)
        distance,ray_cells = self.ray_tracing(start_point, end_point)  
        rospy.loginfo("distence = {}".format(distance))


        markerArray = MarkerArray()
        marker=Marker()
        marker.id=0        
        occupency_values =[]
        map_res = self.map_msg.info.resolution      

        map_orig_x_loc = self.map_msg.info.origin.position.x 
        map_orig_y_loc = self.map_msg.info.origin.position.y    

        for i in range(0,len(ray_cells)):
            x = ray_cells[i][0]
            y = ray_cells[i][1]   
            X_w = self.map_msg.info.origin.position.x + x*self.map_msg.info.resolution  
            Y_w = self.map_msg.info.origin.position.y + y*self.map_msg.info.resolution                
            Xp =[X_w,Y_w]
            marker = self.draw_marker((ray_cells[i][0]*map_res) +map_orig_x_loc ,(ray_cells[i][1]*map_res)+map_orig_y_loc , [0.4,0.1,0.5],"sphere", 0.1)                  
            marker.id = i       
            markerArray.markers.append(marker)  
            occupency_values.append(gridValue(self.map_msg, Xp)) 
        self.markerArray_pub.publish(markerArray)                

        markerArray = MarkerArray()
        map_res = self.map_msg.info.resolution        
        map_orig_x_loc = self.map_msg.info.origin.position.x 
        map_orig_y_loc = self.map_msg.info.origin.position.y
        marker=Marker()
        marker.id=0
        robot_position = self.getrobotposition()

        frontierposx= int((frontier_plan.frontier_loc.x  - self.map_msg.info.origin.position.x) / self.map_msg.info.resolution)
        frontierposy= int((frontier_plan.frontier_loc.y  - self.map_msg.info.origin.position.y) / self.map_msg.info.resolution)     

        robotposx= int((robot_position[0]  - self.map_msg.info.origin.position.x) / self.map_msg.info.resolution)
        robotposy= int((robot_position[1] - self.map_msg.info.origin.position.y) / self.map_msg.info.resolution)  

        rospy.loginfo("Distance between the Robot and Frontier in pixels  is {}".format(self.euclidean_distance(robotposx,robotposy, frontierposx, frontierposy)))        
                
        occupancy_list = []
        try: 
           for occupancy_value in occupency_values:   
                occupancy_list.append(occupancy_value)                            
                if occupancy_value == -1:
                    prob=0.100  # yeilds low entropy and high uncertinaty and low information gain
                elif occupancy_value == 0 or occupancy_value == 100:
                    prob=0.450 #yields high entropy and low uncertinaty and high information gain
                else:
                    rospy.loginfo("I got a different occupancy value of {}".format(occupancy_value))                               
                try:                        
                    entropy +=  (-((prob * math.log2(prob) + (1 - prob)*math.log2(1 - prob)))) #* np.exp(-0.25 *  self.euclidean_distance(robotposx,robotposy, frontierposx, frontierposy))     
                    
                except Exception as e:                        
                    rospy.logerr("problem computing the entropy {}".format(e))                
            # Do something with the occupancy value         
        except Exception as e:
            rospy.logerr("Error processing compute entropy method: {}".format(e))

        #rospy.loginfo("--------------- entropy is  --------------    : {}".format(entropy))
        #self.markerArray_pub.publish(markerArray)   
        if entropy !=0 and len(occupency_values)!=0:    
            entropy = entropy/len(occupency_values)   

        infogain = frontier_plan.spanning_trees - entropy*100     

        #rospy.loginfo("occupancy_list has  {} no. of -1".format(occupancy_list.count(-1)))
        #rospy.loginfo("occupancy_list has  {} no. of 0".format(occupancy_list.count(0)))
        #rospy.loginfo("occupancy_list has  {} no. of 100".format(occupancy_list.count(100)))
        return entropy*100,infogain
    
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
            rospy.logerr("Unable to write CSV file: %s", str(e))

        finally:
            file.close()

    def euclidean_distance(self, x1, y1, x2, y2):
        distance = math.sqrt(math.pow(x2 - x1, 2) + math.pow(y2 - y1, 2))        
        return distance
                               
    def frontiercallback(self,data):
        markerArray = MarkerArray()
        marker = Marker()
        frontier_infogain = FrontierWithPath()

        Froniter_Pose = Pose()
        frontier_plan = data   
        #path_length = len(frontier_plan.waypoints)
        got_the_entropy, infogain = self.compute_path_entropy(frontier_plan) 
            #distance = self.euclidean_distance(frontier_plan.robotxy_pos.x, frontier_plan.robotxy_pos.y, frontier_plan.frontier_loc.x, frontier_plan.frontier_loc.y)
            #utility = got_the_entropy#*distance     
        #rospy.loginfo("------------- Entropy for the frontier {} is {}  \n ".format(frontier_plan.ID,got_the_entropy))
        #rospy.loginfo("------------- Infogain for the frontier {} is {}  \n ".format(frontier_plan.ID,infogain ))

        marker = self.draw_marker(frontier_plan.frontier_loc.x,frontier_plan.frontier_loc.y, [0.5,0.5,1],"sphere", 0.7)        
        self.marker_pub.publish(marker)

        frontier_infogain.ID = frontier_plan.ID
        frontier_infogain.spanning_trees =frontier_plan.spanning_trees
        frontier_infogain.infogain = infogain
        frontier_infogain.entropy = got_the_entropy

        self.frontier_infongain.publish(frontier_infogain)
        #self.markerArray_pub.publish(markerArray)
       

    def callback(self, frontier):
        pass

    
    
if __name__ == '__main__':
    try:
        rospy.init_node('entropy', anonymous=True)
        ComputeEntropy()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

