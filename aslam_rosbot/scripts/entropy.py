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
       
        marker_pos_x = int(self.frontier_plan.robotxy_pos.x)
        marker_pos_y = int(self.frontier_plan.robotxy_pos.y)   



        #print('Ray cells:', ray_cells)
        
        #marker_pos_x = int((marker_pos_x- self.map_msg.info.origin.position.x) / self.map_msg.info.resolution)
        #marker_pos_y  = int((marker_pos_y- self.map_msg.info.origin.position.x) / self.map_msg.info.resolution)
              
        #rospy.loginfo("{}  marker_pos_x ".format(marker_pos_x))
        #rospy.loginfo("{}  marker_pos_y ".format(marker_pos_y ))  
        #      
        marker_posd_x = int(self.chosenfrontier_posx)
        marker_posd_y = int(self.chosenfrontier_posy)


        markerArray = MarkerArray()
        #points = bresenham(self.map_msg.data, int(data.robotxy_pos.x), int(data.robotxy_pos.y), int(self.chosenfrontier_posx), int(self.chosenfrontier_posy))     
        #rospy.loginfo("I got {} points from Breshenam ".format(len(points[0])))         
                           
        #points_xy =  np.zeros((2, int(len(points[0]))))

        #for i in range(len(points[0])):              
            #points_xy[0][i] = points[0][i]
            #points_xy[1][i] = points[1][i]
            #marker = self.draw_marker(points_xy[0][i],points_xy[1][i], [0.5,0.5,0.7],"cube",0.3)
            #marker.id= i                
            #markerArray.markers.append(marker)  
        #self.markerArray_pub.publish(markerArray)  
        #Xp=[data.chosenfrontierxy_pos.x  ,data.chosenfrontierxy_pos.y]
        
        
        #Xp=[data.robotxy_pos.x,data.robotxy_pos.y]            
        
        #Xp_pixels_x = int((Xp[0] - self.map_msg.info.origin.position.x) / self.map_msg.info.resolution)
        #Xp_pixels_y = int((Xp[1] - self.map_msg.info.origin.position.x) / self.map_msg.info.resolution)       

        #rospy.loginfo("------- Xp is  {},{} ----------------".format(Xp[0],Xp[1]))          
        #rospy.loginfo("------- Xp in pixels is  {},{} ----------------".format(Xp_pixels_x,Xp_pixels_y))                            
        
        marker = Marker()
        #value=gridValue(self.map_msg, Xp)        
        #rospy.loginfo("------- occupancy_value  at Xp is  {} ----------------".format(value))          
        marker = self.draw_marker(data.chosenfrontierxy_pos.x,data.chosenfrontierxy_pos.y, [0.5,0.1,0.7],"cube",0.8)
        marker.id= 200
        self.marker_pub.publish(marker) 


        #i = int((marker_pos_x- self.map_msg.info.origin.position.x) / self.map_msg.info.resolution)
        #j = int((marker_pos_y- self.map_msg.info.origin.position.y) / self.map_msg.info.resolution)  
        #if Aloo[0] >= 0 and Aloo[0] < self.map_msg.info.width and Aloo[1] >= 0 and Aloo[1] < self.map_msg.info.height:
        # Extract the occupancy value at the desired location
            #occupancy_value = self.map_msg.data[Aloo[0] + Aloo[1] * self.map_msg.info.width]
            #rospy.loginfo("------- occupancy_value is  {} ----------------".format(occupancy_value ))


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
        return ray_cells
    
    
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
        rospy.loginfo("------------- Computing the entropy for the frontier = {}  ----------------".format(frontier_plan.ID)) 

        robot_position = self.getrobotposition()
        entropy=0        

        start_point = Point( robot_position[0] ,  robot_position[1] , 0)
        end_point = Point( frontier_plan.frontier_loc.x , frontier_plan.frontier_loc.y, 0)
        ray_cells = self.ray_tracing(start_point, end_point)  

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

        #robot_pos_x_pixls = int((frontier_plan.robotxy_pos.x- self.map_msg.info.origin.position.x) / self.map_msg.info.resolution)
        #robot_pos_y_pixls = int((frontier_plan.robotxy_pos.y- self.map_msg.info.origin.position.x) / self.map_msg.info.resolution)


        #for i in range(0,len(ray_cells)):#, int(len(ray_cells[0])/10)):
            #marker = self.draw_marker((ray_cells[i][0]*map_res) +map_orig_x_loc ,(ray_cells[i][1]*map_res)+map_orig_y_loc , [0.4,0.1,0.5],"sphere", 0.1)                  
            #marker.id = i       
            #markerArray.markers.append(marker)  
        #self.markerArray_pub.publish(markerArray)       

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
                    entropy = entropy/len(occupency_values)
                except Exception as e:                        
                    rospy.logerr("problem computing the entropy {}".format(e))                
            # Do something with the occupancy value         
        except Exception as e:
            rospy.logerr("Error processing compute entropy method: {}".format(e))

        #rospy.loginfo("--------------- entropy is  --------------    : {}".format(entropy))
        #self.markerArray_pub.publish(markerArray)       

        rospy.loginfo("occupancy_list has  {} no. of -1".format(occupancy_list.count(-1)))
        rospy.loginfo("occupancy_list has  {} no. of 0".format(occupancy_list.count(0)))
        rospy.loginfo("occupancy_list has  {} no. of 100".format(occupancy_list.count(100)))
        return entropy
    
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

        Froniter_Pose = Pose()
        frontier_plan = data   
        path_length = len(frontier_plan.waypoints)       

        #rospy.loginfo("frontier_plan.robotxy_pos.x = {}".format(frontier_plan.robotxy_pos.x))
        #rospy.loginfo("frontier_plan.robotxy_pos.y = {}".format(frontier_plan.robotxy_pos.y))
        
    
        #rospy.loginfo(" -------------- Robot is at x,y = {},{}----------------".format(frontier_plan.robotxy_pos.x, frontier_plan.robotxy_pos.y))
        #rospy.loginfo(" -------------- Frontier is at x,y = {},{}----------------".format(frontier_plan.frontier_loc.x, frontier_plan.frontier_loc.y))

        #
        #rospy.loginfo(" -------- Path length is ----------= {} ".format(path_length ))

        #posematrix =  np.zeros((2, int(path_length)))
        #rr,cc = np.shape(posematrix) 
        #rospy.loginfo(" --------------Got frointier with ID = {} --- and name {} ----------------".format(self.frontier_plan.ID, self.frontier_plan.name))
        #rospy.loginfo(" -------- I will process plan for  ----------= {} frontiers".format(self.frontier_plan.totalfrontiers))
        #plan[i].pose.position.x
        #rospy.loginfo(" -------- self.frontier_plan_  ----------= {}".format(len(self.frontier_plan.waypoints)))
        #self.Froniter_Pose.position=self.frontier_plan_[0].position  
        

        got_the_entropy = self.compute_path_entropy(frontier_plan) 
            #distance = self.euclidean_distance(frontier_plan.robotxy_pos.x, frontier_plan.robotxy_pos.y, frontier_plan.frontier_loc.x, frontier_plan.frontier_loc.y)
            #utility = got_the_entropy#*distance     
        rospy.loginfo("------------- Raycasting Entropy for the frontier {} is {}  \n ".format(frontier_plan.ID,got_the_entropy))
            #rospy.loginfo("Utility of frontirer  {} is  {} \n ".format(frontier_plan.ID,utility))

        #rospy.loginfo("--------- markers are  {} -------- ".format(markerArray))
        #self.compute_path_entropy(self.Froniter_Pose) 
        #rospy.loginfo("Got frontier plan with length  {}".format(self.frontier_plan_.name))
        #rospy.loginfo("Got frontier at pos x = {}".format(self.frontier_plan.frontier_loc.x))
        #rospy.loginfo("Got frontier at pos y = {}".format(self.frontier_plan.frontier_loc.y))
        #def draw_marker(self,x, y, color=[1.0,0.0,0.0], mtype="sphere", scale=0.1, ns='my_marker_ns'):
        marker = self.draw_marker(frontier_plan.frontier_loc.x,frontier_plan.frontier_loc.y, [0.5,0.5,1],"sphere", 0.7)        
        self.marker_pub.publish(marker)
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

