#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleDetector:
    def __init__(self):
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.twist_msg = Twist()

    def scan_callback(self, scan_msg):
        # process the scan data to detect obstacles
        # if an obstacle is detected, set the twist_msg to stop the robot
        if min(scan_msg.ranges) < 0.5:
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = 0.0
        else:
            self.twist_msg.linear.x = 0.5
            self.twist_msg.angular.z = 0.0

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.cmd_vel_pub.publish(self.twist_msg)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('obstacle_detector')
    obstacle_detector = ObstacleDetector()
    obstacle_detector.run()

