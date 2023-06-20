#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
import actionlib
from tf.transformations import euler_from_quaternion
from math import atan2, sqrt, cos, sin


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

        # Store the positions and orientations of all robots
        self.robot_positions = {}
        self.robot_orientations = {}

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

    def get_robot_positions(self):
        # Retrieve the positions of all robots from the move_base/current_goal topic
        try:
            positions = rospy.wait_for_message('move_base/current_goal', PoseStamped, timeout=1)
            for position in positions:
                robot_id = position.header.frame_id  # Assuming each robot has a unique frame_id
                self.robot_positions[robot_id] = position.pose.position
                self.robot_orientations[robot_id] = position.pose.orientation
        except rospy.ROSException:
            rospy.logwarn("Failed to get robot positions")

    def calculate_distance(self, x1, y1, x2, y2):
        # Calculate the Euclidean distance between two points
        return sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def calculate_angle(self, x1, y1, x2, y2):
        # Calculate the angle between two points
        return atan2(y2 - y1, x2 - x1)

    def avoid_collision(self, ranges):
        # Set a threshold distance for obstacle detection
        obstacle_distance_threshold = 0.5

        obstacle_detected = any(distance < obstacle_distance_threshold for distance in ranges)

        if obstacle_detected:
            rospy.loginfo("Obstacle detected!")
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

    def laser_callback(self, msg):
        # Get the range data from the laser scan message
        ranges = msg.ranges

        # Avoid collision with obstacles
        self.avoid_collision(ranges)

        # Get the positions of all other robots
        self.get_robot_positions()

        if self.robot_positions:
            for robot_id, position in self.robot_positions.items():
                if robot_id != self.current_robot_id:
                    # Calculate the distance between the current robot and the other robots
                    distance = self.calculate_distance(position.x, position.y,
                                                       self.current_goal.pose.position.x,
                                                       self.current_goal.pose.position.y)

                    if distance < 1.0:
                        # Calculate the angle between the current robot and the other robots
                        angle = self.calculate_angle(position.x, position.y,
                                                     self.current_goal.pose.position.x,
                                                     self.current_goal.pose.position.y)

                        # Adjust the current goal based on the positions of the other robots
                        adjusted_goal = MoveBaseGoal()
                        adjusted_goal.target_pose.header.frame_id = "map"
                        adjusted_goal.target_pose.header.stamp = rospy.Time.now()
                        adjusted_goal.target_pose.pose.position.x = self.current_goal.pose.position.x + 0.5 * cos(angle)
                        adjusted_goal.target_pose.pose.position.y = self.current_goal.pose.position.y + 0.5 * sin(angle)
                        adjusted_goal.target_pose.pose.orientation = self.current_goal.pose.orientation

                        self.send_goal(adjusted_goal)
                        self.received = False

        else:
            rospy.logwarn("Failed to get other robots' positions")


if __name__ == '__main__':
    try:
        node = CollisionAvoidanceNode()
    except rospy.ROSInterruptException:
        pass
