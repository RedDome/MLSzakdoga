import gymnasium as gym
import rospy
import numpy as np
from gymnasium import spaces
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion

class CustomGazeboEnv(gym.Env):
    def __init__(self, start_position=(0, 0), goal_position=(5, 5)):
        super(CustomGazeboEnv, self).__init__()
        
        self.start_position = np.array(start_position, dtype=np.float32)
        self.goal_position = np.array(goal_position, dtype=np.float32)
        
        self.cmd_vel_pub = rospy.Publisher('/turtlebot3_burger/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/turtlebot3_burger/odom', Odometry, self._odom_callback)
        self.laser_sub = rospy.Subscriber('/turtlebot3_burger/scan', LaserScan, self._laser_callback)
        
        self.action_space = spaces.Discrete(3)  # 0: Forward, 1: Left, 2: Right
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(5,), dtype=np.float32)
        
        self.robot_position = np.array(self.start_position, dtype=np.float32)
        self.robot_orientation = 0  # Orientation in radians
        
        self.reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

    def _odom_callback(self, data):
        """Callback function to update robot position and orientation from odometry data."""
        self.robot_position = np.array([data.pose.pose.position.x, data.pose.pose.position.y], dtype=np.float32)
        orientation_q = data.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.robot_orientation = yaw
        # print("o:",self.robot_position)

    def _laser_callback(self, data):
        """Callback function to update laser scan data."""
        self.laser_data = np.array(data.ranges, dtype=np.float32)
        print("s:", self.laser_data)

    def reset(self, seed=None, options=None):
        """Reset the environment to the starting position."""
        super().reset(seed=seed)
        try:
            self.reset_simulation()
            rospy.sleep(1)
        except rospy.exceptions.ROSInterruptException:
            pass
        
        self.robot_position = np.array(self.start_position, dtype=np.float32)
        self.robot_orientation = 0
        self.laser_data = np.zeros(1, dtype=np.float32)
        obs = self._get_obs()
        return obs, {}

    def _get_obs(self):
        """Get the current observation, including position, orientation, and laser data."""
        distance_to_goal = np.linalg.norm(self.goal_position - self.robot_position).astype(np.float32)
        angle_to_goal = np.arctan2(self.goal_position[1] - self.robot_position[1], self.goal_position[0] - self.robot_position[0]).astype(np.float32)
        laser_min_distance = np.min(self.laser_data).astype(np.float32)
        return np.array([self.robot_position[0], self.robot_position[1], distance_to_goal, angle_to_goal, laser_min_distance], dtype=np.float32)

    def reward_function(self):
        """Calculate reward based on the distance to the goal and obstacle avoidance."""
        distance_to_goal = np.linalg.norm(self.goal_position - self.robot_position)

        # Reward for getting closer to the goal
        if distance_to_goal < 0.1:
            reward = 100.0
        else:
            reward = -float(distance_to_goal)  # Negative reward based on distance to goal
        
        # Penalty for being too close to obstacles
        if self.laser_data is not None:
            min_distance = np.min(self.laser_data)
            if min_distance < 0.5:
                reward -= 10 * (0.5 - min_distance)  # Higher penalty the closer it is to an obstacle
        
        return reward

    def step(self, action):
        """Take a step in the environment based on the action chosen."""
        vel_msg = Twist()
        if action == 0:  # Forward
            vel_msg.linear.x = 0.2 # 2. gyorsabb
            vel_msg.angular.z = 0.0
        elif action == 1:  # Left turn
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.3 # 3. gyorsabb
        elif action == 2:  # Right turn
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = -0.3 # 3. gyorsabb
        # Publish velocity command
        self.cmd_vel_pub.publish(vel_msg)
        try:
            rospy.sleep(0.1)
        except rospy.exceptions.ROSInterruptException:
            pass
        
        obs = self._get_obs()
        reward = self.reward_function()
        
        # Check if goal is reached
        terminated = np.linalg.norm(self.goal_position - self.robot_position) < 0.1
        done = bool(terminated)
        
        return obs, float(reward), done, False, {}

    def close(self):
        """Clean up ROS-related resources."""
        rospy.signal_shutdown('Environment closed.')

    def set_goal_position(self, x, y):
        """Set a new goal position for the robot."""
        self.goal_position = np.array([x, y], dtype=np.float32)
