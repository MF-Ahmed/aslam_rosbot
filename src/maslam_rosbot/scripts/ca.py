#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
import actionlib


class CollisionAvoidanceNode:
    def __init__(self):
        rospy.init_node('collision_avoidance')

        # Subscriber to retrieve the scan information
        self.laser_sub = rospy.Subscriber('scan', LaserScan, self.laser_callback)
        # Subscriber to retrieve the move base goal information
        self.goal_sub = rospy.Subscriber('move_base/goal', MoveBaseActionGoal, self.goal_callback)

        # Other initialization code

        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()

        # Variable to store the current goal
        self.current_goal = None
        # Variable to store if the goal has been received
        self.received = False

        rospy.spin()

    def send_goal(self, goal):
        self.current_goal = goal  # Store the current goal
        self.move_base_client.send_goal(goal)

    def stop_robot(self):
        # Stop the robot by sending a zero velocity command
        cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        cmd_vel_msg = Twist()
        cmd_vel_pub.publish(cmd_vel_msg)

    def goal_callback(self, msg):
        if not self.received:
            self.current_goal = msg.goal.target_pose
            self.received = True

    def fill_goal(self, goal):
        goal.target_pose = self.current_goal
        goal.target_pose.header.stamp = rospy.Time.now()  # Add header timestamp
        return goal

    def clear_costmaps(self):
        rospy.wait_for_service('move_base/clear_costmaps')
        try:
            clear_costmaps_service = rospy.ServiceProxy('move_base/clear_costmaps', Empty)
            clear_costmaps_service()
        except rospy.ServiceException as e:
            rospy.logwarn(f"Failed to clear costmaps: {e}")

    def laser_callback(self, msg):
        # Get the range data from the laser scan message
        ranges = msg.ranges

        # Set a threshold distance for obstacle detection
        obstacle_distance_threshold = 0.5

        obstacle_detected = any(distance < obstacle_distance_threshold for distance in ranges)

        # If an unexpected obstacle is detected:
        if obstacle_detected:
            rospy.loginfo("Obstacles detected!")
            self.stop_robot()

            if self.current_goal is not None:
                # Stop the robot
                self.move_base_client.cancel_all_goals()

                # Clear costmaps
                self.clear_costmaps()

                # Replan for the same goal
                new_goal = MoveBaseGoal()
                new_goal = self.fill_goal(new_goal)

                self.send_goal(new_goal)
                self.received = False

            else:
                rospy.loginfo("No goal was set")

if __name__ == '__main__':
    try:
        node = CollisionAvoidanceNode()
    except rospy.ROSInterruptException:
        pass
