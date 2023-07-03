#!/usr/bin/env python3
import rospy
import rospy
from std_msgs.msg import Float32
import matplotlib
matplotlib.use('Agg')  # Set the backend to Agg
import matplotlib.pyplot as plt
import datetime
'''
Plots the No. of Spanning trees criteria vs the time 
reades from the topic to get the no of spanning trees 
'''

legendflag=True

def callback(data):
    values.append(data.data)
    times.append(rospy.Time.now().to_sec())  # Store the current ROS time
    
    # Plot the values as connected markers with enhanced aesthetics
    plt.plot(times, values, 'o-', color='blue', linewidth=2, markersize=6,
             markerfacecolor='white', markeredgewidth=1.5, markeredgecolor='blue')#, label='Our Approach')
    plt.ylim(1000, 5000) 
    plt.xlabel('Time', fontsize=12)
    plt.ylabel('No. of Spanning trees', fontsize=12)
    plt.title('The Evolution of D optimality Critera Ours ', fontsize=14, fontweight='bold')
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.xticks(fontsize=10)
    plt.yticks(fontsize=10)
    #plt.legend()      
    plt.tight_layout()  # Improve spacing between subplots

    #current_time = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    #file_name = f"/home/usr/data/catkin_ws/src/aslam_rosbot/results/d_opti_graph_{current_time}.png"
    file_name = f"/home/usr/data/catkin_ws/src/aslam_rosbot/results/d_opti_graph.png"
    plt.savefig(file_name, dpi=300)  


if __name__ == '__main__':
    rospy.init_node('d_opti_plot')
    rospy.loginfo("d_opti_plot node initialized")
    
    topic = rospy.get_param('~topic', '/d_optimality_publisher/Dopt/')
    values = []  # Store the received values
    times =[]    


    rospy.Subscriber(topic, Float32, callback)

    rospy.spin()
