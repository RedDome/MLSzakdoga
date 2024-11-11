#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math

class TurtleBot3Drive:
    def __init__(self):
        rospy.init_node('turtlebot3_drive')

        # Initialize parameters
        self.cmd_vel_topic_name = rospy.get_param("~cmd_vel_topic_name", "cmd_vel")
        self.escape_range = math.radians(30.0)
        self.check_forward_dist = 0.7
        self.check_side_dist = 0.6

        # Initialize variables
        self.tb3_pose = 0.0
        self.prev_tb3_pose = 0.0
        self.scan_data = [0, 0, 0]

        # Initialize publishers and subscribers
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic_name, Twist, queue_size=10)
        rospy.Subscriber("scan", LaserScan, self.laser_scan_callback)
        rospy.Subscriber("odom", Odometry, self.odom_callback)

        rospy.loginfo("TurtleBot3 Simulation Node Init")

    def odom_callback(self, msg):
        siny = 2.0 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z +
                      msg.pose.pose.orientation.x * msg.pose.pose.orientation.y)
        cosy = 1.0 - 2.0 * (msg.pose.pose.orientation.y**2 + msg.pose.pose.orientation.z**2)
        self.tb3_pose = math.atan2(siny, cosy)

    def laser_scan_callback(self, msg):
        scan_angle = [0, 30, 330]
        for i, angle in enumerate(scan_angle):
            if math.isinf(msg.ranges[angle]):
                self.scan_data[i] = msg.range_max
            else:
                self.scan_data[i] = msg.ranges[angle]

    def update_command_velocity(self, linear, angular):
        cmd_vel = Twist()
        cmd_vel.linear.x = linear
        cmd_vel.angular.z = angular
        self.cmd_vel_pub.publish(cmd_vel)

    def control_loop(self):
        state = "GET_TB3_DIRECTION"

        if state == "GET_TB3_DIRECTION":
            if self.scan_data[1] > self.check_forward_dist:
                if self.scan_data[0] < self.check_side_dist:
                    self.prev_tb3_pose = self.tb3_pose
                    state = "TB3_RIGHT_TURN"
                elif self.scan_data[2] < self.check_side_dist:
                    self.prev_tb3_pose = self.tb3_pose
                    state = "TB3_LEFT_TURN"
                else:
                    state = "TB3_DRIVE_FORWARD"

            if self.scan_data[1] < self.check_forward_dist:
                self.prev_tb3_pose = self.tb3_pose
                state = "TB3_RIGHT_TURN"

        elif state == "TB3_DRIVE_FORWARD":
            self.update_command_velocity(0.2, 0.0)
            state = "GET_TB3_DIRECTION"

        elif state == "TB3_RIGHT_TURN":
            if abs(self.prev_tb3_pose - self.tb3_pose) >= self.escape_range:
                state = "GET_TB3_DIRECTION"
            else:
                self.update_command_velocity(0.0, -0.5)

        elif state == "TB3_LEFT_TURN":
            if abs(self.prev_tb3_pose - self.tb3_pose) >= self.escape_range:
                state = "GET_TB3_DIRECTION"
            else:
                self.update_command_velocity(0.0, 0.5)

    def run(self):
        rate = rospy.Rate(125)
        while not rospy.is_shutdown():
            self.control_loop()
            rate.sleep()

if __name__ == '__main__':
    try:
        turtlebot3_drive = TurtleBot3Drive()
        turtlebot3_drive.run()
    except rospy.ROSInterruptException:
        pass
