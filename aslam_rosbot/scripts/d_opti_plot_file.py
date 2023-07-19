#!/usr/bin/env python3
from std_msgs.msg import Float32
import matplotlib
matplotlib.use('Agg')  # Set the backend to Agg
import matplotlib.pyplot as plt
import datetime
import numpy as np

'''
Plots the No. of Spanning trees criteria vs the time 
reads from the file at specific intervals and plots the no of spanning trees evolution
'''
GRAPH_PATH_ = "/home/usr/data/catkin_ws/src/aslam_rosbot/aslam_rosbot/maps/pose_graph.g2o"
opt_total2=0
values = []  # Store the received values
times =[]  
goal_list =[]

def goalcallback(data):
    rospy.loginfo("Received goal from explorer node: x = %f, y = %f", data.x, data.y)
    goal_list.append([data.x, data.y])
    rospy.loginfo("Received {} goals".format(len(goal_list)))
    nodes, edges = getGraph(GRAPH_PATH_) # reads from the .g2o  pose graph file 
    opti_total= read_opti (edges)   
    plot(opti_total)

def goaltimecallback(data):
    pass
    #rospy.loginfo("Received goal time form explorer node: {}".format(data.data.secs))
    #goal_list.append([data.x, data.y])
    #rospy.loginfo("Received {} goals".format(len(goal_list)))
    #nodes, edges = getGraph(GRAPH_PATH_) # reads from the .g2o  pose graph file 
    #opti_total= read_opti (edges)   
    #plot(opti_total,data.data.secs)

def getGraph(filename):
    nodes = []
    edges = []
    with open(filename) as fp:
        lines = fp.readlines()
        for x in lines:
            edge_type = x.split(' ')[0]
            if edge_type == "VERTEX_SE2":
                node = np.float_(x.split(' ')[1])  # node name 
                pose = np.float_(x.split(' ')[2:5]) # node pose
                nodes.append(np.concatenate([np.array([node]), pose]).ravel())
            elif edge_type == "EDGE_SE2":
                node1 = np.float_(x.split(' ')[1]) # from node
                node2 = np.float_(x.split(' ')[2]) # to node
                etype = 0 if (abs(node1 - node2) == 1) else 1
                delta = np.float_(x.split(' ')[3:6]) # three entries
                FIM = np.float_(x.split(' ')[7:13]) # six entries because g2o saves the infromation matrix
                #as an upper triangular form  serialization is q_11, q_12, q_13, q_22, q_23, q_33
                #rospy.loginfo("FIM is {}".format(FIM))
                edges.append(np.concatenate([np.array([node1, node2, etype]), delta, FIM]).ravel())

    return nodes, edges

def read_opti (edges):

    opt_total=0
    opt_total2=0
    
    if edges is not []:
            for i in range(0, np.size(edges, 0)):
                edge = (edges[i][0], edges[i][1])
                I = edges[i][6:12]  # six members
                #rospy.loginfo("I =  {}".format(I))
                A = [[I[0], I[1], I[2]],
                [I[1], I[3], I[4]],
                [I[2], I[4], I[5]]]
                eigv2 = np.linalg.eigvals(A)
                eigv = eigv2[eigv2 > 1e-8]

                n = np.size(A, 1)                
                opt_cri = np.exp(np.sum(np.log(eigv)) / n)      
                opt_total2 = opt_total2 + opt_cri  
            opt_total = opt_total2/np.size(edges, 0)   
    return opt_total


def plot(data):
    values.append(data)
    times.append(range(len(values)))
    #times.append(rospy.Time.now().to_sec())  # Store the current ROS time
    #times.append(time)
    # Plot the values
    #plt.plot(range(len(values)), values, 'o-', color='b')
    
    # Plot the values as connected markers with enhanced aesthetics
    plt.plot(times, values, '.-', color='teal', linewidth=2, markersize=6,
             markerfacecolor='white', markeredgewidth=1.2, markeredgecolor='teal')
    plt.ylim(1000,5000)
    plt.xlabel('Time', fontsize=12)
    plt.ylabel('No. of Spanning trees', fontsize=12)
    plt.title('The Evolution of D optimality Critera ', fontsize=14, fontweight='bold')
    plt.grid(True, linestyle='--', alpha=0.5)
    plt.xticks(fontsize=10)
    plt.yticks(fontsize=10)
    plt.tight_layout()  # Improve spacing between subplots
    #current_time = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    #file_name = f"/home/usr/data/catkin_ws/src/aslam_turtlebot/results/d_opti_graph_{current_time}.png"
    file_name = f"/home/usr/data/catkin_ws/src/aslam_rosbot/results/d_opti_graph.png"
    
    plt.savefig(file_name, dpi=300)  


if __name__ == '__main__':
 
    nodes, edges = getGraph(GRAPH_PATH_) # reads from the .g2o  pose graph file 
    opt_total = read_opti(edges)
    plot(opt_total)