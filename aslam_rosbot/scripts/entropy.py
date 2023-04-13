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
       


        start_point = Point(data.robotxy_pos.x, data.robotxy_pos.y, 0)
        end_point = Point(data.chosenfrontierxy_pos.x, data.chosenfrontierxy_pos.y, 0)

        ray_cells, occupency_values = self.ray_tracing(start_point, end_point)
        rospy.loginfo("------- ray_cells ----------------{}".format(ray_cells)) 
        rospy.loginfo("------- ray_cells shape ----------------{}".format(len(ray_cells))) 

        rospy.loginfo("------- occupency_values ----------------{}".format(occupency_values)) 
        rospy.loginfo("------- occupency_values ----------------{}".format(len(occupency_values)))         

        new_map_data = self.draw_ray_on_map(ray_cells)
        rospy.loginfo("------- new_map_data ----------------{}".format(new_map_data)) 
        rospy.loginfo("------- new_map_data shape is {} ".format(new_map_data.shape)) 
        

        #self.plot_map_with_markers(map_data)



        #print('Ray cells:', ray_cells)
        
        #marker_pos_x = int((marker_pos_x- self.map_msg.info.origin.position.x) / self.map_msg.info.resolution)
        #marker_pos_y  = int((marker_pos_y- self.map_msg.info.origin.position.x) / self.map_msg.info.resolution)
              
        #rospy.loginfo("{}  marker_pos_x ".format(marker_pos_x))
        #rospy.loginfo("{}  marker_pos_y ".format(marker_pos_y ))  
        #      
        marker_posd_x = int(self.chosenfrontier_posx)
        marker_posd_y = int(self.chosenfrontier_posy)

        #marker_posd_x = int((marker_posd_x - self.map_msg.info.origin.position.x) / self.map_msg.info.resolution)
        #marker_posd_y  = int((marker_posd_y - self.map_msg.info.origin.position.x) / self.map_msg.info.resolution)       

        # Returns the mapData's index of a given point (x,y) in the map
        #def index_of_point(mapData, Xp):

        # Returns the point (x,y) of a given mapData's index
        #
        # def point_of_index(mapData, i):    
        #Aloo = point_of_index(self.map_msg, marker_posd_x )

        #rospy.loginfo("index of marker_posd_x is {},{}  ".format(Aloo[0],format(Aloo[1])))
        #index_of_point()
        #point_of_index()

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
        
        
        Xp=[data.robotxy_pos.x,data.robotxy_pos.y]            
        
        Xp_pixels_x = int((Xp[0] - self.map_msg.info.origin.position.x) / self.map_msg.info.resolution)
        Xp_pixels_y = int((Xp[1] - self.map_msg.info.origin.position.x) / self.map_msg.info.resolution)       

        rospy.loginfo("------- Xp is  {},{} ----------------".format(Xp[0],Xp[1]))          
        rospy.loginfo("------- Xp in pixels is  {},{} ----------------".format(Xp_pixels_x,Xp_pixels_y))                            
        
        marker = Marker()
        value=gridValue(self.map_msg, Xp)        
        rospy.loginfo("------- occupancy_value  at Xp is  {} ----------------".format(value))          
        marker = self.draw_marker(data.chosenfrontierxy_pos.x,data.chosenfrontierxy_pos.y, [0.5,0.1,0.7],"cube",0.6)
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
        #rospy.loginfo("in path callback")
        #rospy.loginfo("Path format is  = {}".format(data))        
        #print([x.pose.position.x for x in data.poses])
        #print([y.pose.position.y for y in data.poses])
        # Your path planning code goes here

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
        cell_x = int((point.x - map_origin[0]) / map_resolution)
        cell_y = int((point.y - map_origin[1]) / map_resolution)
        return (cell_x, cell_y)

    def ray_tracing(self,start_point, end_point):
        """ Perform ray tracing from start_point to end_point """
        start_cell = self.get_map_cell(start_point)
        end_cell = self.get_map_cell(end_point)
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
        occupancy_values = [map_data[cell[0], cell[1]] for cell in ray_cells]       
        return ray_cells,occupancy_values         
    

    def draw_ray_on_map(self,ray_cells):
        """ Draw the ray on the occupancy grid map """
        global map_data
        new_map_data = np.copy(map_data)
        for cell in ray_cells:
            if cell[0] < 0 or cell[0] >= map_width or cell[1] < 0 or cell[1] >= map_height:
                continue
            new_map_data[cell[1], cell[0]] = 100  # set the occupancy grid value to 100
        return new_map_data
    
    def plot_map_with_markers(self,map_data):
        """ Plot the occupancy grid map with Markers """
        global map_resolution, map_origin
        marker_pub = rospy.Publisher('/visualization_marker2', Marker, queue_size=10)
        marker_msg = Marker()
        marker_msg.header.frame_id = 'map'
        marker_msg.type = Marker.CUBE_LIST
        marker_msg.action = Marker.ADD
        marker_msg.pose.orientation.w = 1.0
        marker_msg.scale.x = map_resolution
        marker_msg.scale.y = map_resolution
        marker_msg.scale.z = map_resolution
        for y in range(map_data.shape[0]):
            for x in range(map_data.shape[1]):
                if map_data[y, x] == 100:  # occupied cell
                    p = Point()
                    p.x = x * map_resolution + map_origin[0]
                    p.y = y * map_resolution + map_origin[1]
                    p.z = 0
                    marker_msg.points.append(p)
                    marker_msg.colors.append(ColorRGBA(0.5, 0.5, 0.5, 1.0))  # gray color
                elif map_data[y, x] == 0:  # free cell
                    p = Point()
                    p.x = x * map_resolution + map_origin[0]
                    p.y = y * map_resolution + map_origin[1]
                    p.z = 0
                    marker_msg.points.append(p)
                    marker_msg.colors.append(ColorRGBA(1.0, 1.0, 1.0, 1.0))  # white color
        marker_pub.publish(marker_msg)
        
       
    def compute_path_entropy(self,pathmatrix,frontierID,frontierposx, frontierposy):        
        self.markerArray = MarkerArray()
        rr,cc= np.shape(pathmatrix)        
        rospy.loginfo("------------- I got the frontier = {}  ----------------".format(frontierID))
        occupancy_value=0
        entropy=0.000        
        #self.path_pose[0].append(x)
        #self.path_pose[1].append(y)
        #rospy.loginfo("sdfsdfsfsdfsfsd=sdfsdfsdf {}".format(self.path_pose))
        try: 
           for k in range(0,cc,20):
                #self.draw_marker(pathmatrix[0,k],pathmatrix[1,k],color=[0.0,10.0,10])
                i = int((pathmatrix[0,k] - self.map_msg.info.origin.position.x) / self.map_msg.info.resolution)
                j = int((pathmatrix[1,k] - self.map_msg.info.origin.position.y) / self.map_msg.info.resolution)            
            # Check if the desired location is within the map bounds
                if i >= 0 and i < self.map_msg.info.width and j >= 0 and j < self.map_msg.info.height:
            # Extract the occupancy value at the desired location
                    #rospy.loginfo("i  = {}".format(i))
                    #rospy.loginfo("j  = {}".format(j))
                    occupancy_value = self.map_msg.data[i + j * self.map_msg.info.width]
                    #self.marker = self.draw_marker(i,j, [1,1,0.6],"cube",1)
                    #self.marker.id= k                
                    #self.markerArray.markers.append(self.marker)

                    if occupancy_value == -1:
                        prob=0.100  # yeilds low entropy and high uncertinaty and low information gain
                        #rospy.loginfo("I got {}".format(occupancy_value))
                        #rospy.loginfo("at location {},{}".format(i,j))
                        #marker = self.draw_marker(i,j, [1,0.5,1],"sphere", 10)  
                        #entropy =  (-(prob * math.log2(prob) + (1 - prob)*math.log2(1 - prob)))  
                        #rospy.loginfo("with entropy =  {}".format(entropy))
                        #marker.id= 1
                        #self.marker_pub.publish(marker)
                        #prob = 0.5
                    elif occupancy_value == 0 or occupancy_value == 100:
                        prob=0.450 #yields high entropy and low uncertinaty and high information gain
                    else:
                        rospy.loginfo("I got a different occupancy value of {}".format(occupancy_value))
                        rospy.loginfo("at location {},{}".format(i,j))                                    
                    try:                        
                        entropy +=  (-((prob * math.log2(prob) + (1 - prob)*math.log2(1 - prob)))) * np.exp(-0.25 *  self.euclidean_distance(i,j, frontierposx, frontierposy))
                        
                    except Exception as e:                        
                        rospy.logerr("problem computing the entropy {}".format(e))                
            # Do something with the occupancy value         
        #rospy.loginfo("Occupancy value at ({}, {}): {}".format(x, y, self.occupancy_value))
        except Exception as e:
            rospy.logerr("Error processing compute entropy method: {}".format(e))

        #rospy.loginfo("--------------- entropy is  --------------    : {}".format(entropy))
        #self.markerArray_pub.publish(markerArray)
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
        distance = distance*self.map_msg.info.resolution
        return distance

                               
    def frontiercallback(self,data):
        markerArray = MarkerArray()
        marker = Marker()

        Froniter_Pose = Pose()
        frontier_plan = data   
        frontier_plan.totalfrontiers  
        path_length = len(frontier_plan.waypoints)
        frontier_plan.robotxy_pos.x 

        #rospy.loginfo("frontier_plan.robotxy_pos.x = {}".format(frontier_plan.robotxy_pos.x))
        #rospy.loginfo("frontier_plan.robotxy_pos.y = {}".format(frontier_plan.robotxy_pos.y))
        
    
        #rospy.loginfo(" -------------- Robot is at x,y = {},{}----------------".format(frontier_plan.robotxy_pos.x, frontier_plan.robotxy_pos.y))
        #rospy.loginfo(" -------------- Frontier is at x,y = {},{}----------------".format(frontier_plan.frontier_loc.x, frontier_plan.frontier_loc.y))

        #
        #rospy.loginfo(" -------- Path length is ----------= {} ".format(path_length ))

        posematrix =  np.zeros((2, int(path_length)))
        rr,cc = np.shape(posematrix) 
        #rospy.loginfo(" --------------Got frointier with ID = {} --- and name {} ----------------".format(self.frontier_plan.ID, self.frontier_plan.name))
        #rospy.loginfo(" -------- I will process plan for  ----------= {} frontiers".format(self.frontier_plan.totalfrontiers))
        #plan[i].pose.position.x
        #rospy.loginfo(" -------- self.frontier_plan_  ----------= {}".format(len(self.frontier_plan.waypoints)))
        #self.Froniter_Pose.position=self.frontier_plan_[0].position        
        
        #for j in range(0, self.frontier_plan.totalfrontiers):
        if frontier_plan != 0 or len(frontier_plan.waypoints) != 0:
            for i in range(0,int(path_length), 30):
                Froniter_Pose.position.x=frontier_plan.waypoints[i].position.x
                Froniter_Pose.position.y=frontier_plan.waypoints[i].position.y
                posematrix[0,i]=Froniter_Pose.position.x
                posematrix[1,i]=Froniter_Pose.position.y
                marker = self.draw_marker(Froniter_Pose.position.x,Froniter_Pose.position.y, [1,0.5,0.6],"cube",0.2)
                #marker.id= i                
                #markerArray.markers.append(marker)
                
                #self.draw_marker(self.Froniter_Pose.position.x,self.Froniter_Pose.position.y,[0.0,10.0,0.0], "sphere",10 )
            self.savetofile(posematrix,frontier_plan.ID)
            red_posematrix=posematrix    
            got_the_entropy = self.compute_path_entropy(red_posematrix,frontier_plan.name, frontier_plan.frontier_loc.x,frontier_plan.frontier_loc.y) 
            #distance = self.euclidean_distance(frontier_plan.robotxy_pos.x, frontier_plan.robotxy_pos.y, frontier_plan.frontier_loc.x, frontier_plan.frontier_loc.y)
            utility = got_the_entropy#*distance     
            rospy.loginfo("Path Entropy for the frontier {} as {}  \n ".format(frontier_plan.ID,got_the_entropy))
            rospy.loginfo("Utility of frontirer  {} is  {} \n ".format(frontier_plan.ID,utility))

        #rospy.loginfo("--------- markers are  {} -------- ".format(markerArray))
        #self.compute_path_entropy(self.Froniter_Pose) 
        #rospy.loginfo("Got frontier plan with length  {}".format(self.frontier_plan_.name))
        #rospy.loginfo("Got frontier at pos x = {}".format(self.frontier_plan.frontier_loc.x))
        #rospy.loginfo("Got frontier at pos y = {}".format(self.frontier_plan.frontier_loc.y))
        #def draw_marker(self,x, y, color=[1.0,0.0,0.0], mtype="sphere", scale=0.1, ns='my_marker_ns'):
        marker = self.draw_marker(frontier_plan.frontier_loc.x,frontier_plan.frontier_loc.y, [0.5,0.5,1],"sphere", 0.5)        
        self.marker_pub.publish(marker)
        #self.markerArray_pub.publish(markerArray)
       

    def callback(self, frontier):
        pass
        #rospy.loginfo("in frontier callback")
        #for i in range(0,len(frontier.points)):
            #pass #rospy.loginfo("\n\r got froniter {} at = {}".format(i, frontier.points[i]))
      
        #plan_service = rospy.get_param('~plan_service', '/move_base_node/NavfnROS/make_plan')
    #rospy.wait_for_service(plan_service)
    

    #
    
    # spin() simply keeps python from exiting until this node is stopped
    
    

if __name__ == '__main__':
    try:
        rospy.init_node('entropy', anonymous=True)
        ComputeEntropy()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


''''
double prob = int(latest_map_msg_.data.at(id));
    prob == -1 ? prob = 50 : prob;
    prob == 0 ? prob = 0.00000000001 : (prob == 100 ? prob = 0.999999999 : prob = prob /
                                                                                100.0);
    entropy[cost] +=
            -(prob * std::log2(prob) + (1 - prob) * std::log2(1 - prob)) *
            std::exp(-0.25 * (dist));
'''

