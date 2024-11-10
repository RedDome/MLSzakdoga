import gymnasium as gym
from gymnasium import spaces
import numpy as np
import rospy
from std_srvs.srv import Empty
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose
import time
from nav_msgs.msg import Odometry

class CustomGazeboEnv(gym.Env):
    def __init__(self):
        super(CustomGazeboEnv, self).__init__()

        # Initialize ROS node
        rospy.init_node('gym_gazebo_env', anonymous=True)

        # Define action and observation space
        self.action_space = spaces.Box(low=np.array([-0.5, -1.0]),
                                       high=np.array([0.5, 1.0]),
                                       dtype=np.float32)

        self.observation_space = spaces.Box(low=0.0,
                                            high=1.0,
                                            shape=(180,),
                                            dtype=np.float32)

        # Publishers and Subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self._scan_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self._odom_callback)
        self.reset_simulation_service = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.unpause_physics_client = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause_physics_client = rospy.ServiceProxy('/gazebo/pause_physics', Empty)

        self.scan_data = None
        self.robot_position = None
        self.previous_goal_distance = float('inf')

        # Define goal position (x, y)
        self.goal_position = np.array([5.0, 5.0])  # Default goal coordinates

    def _scan_callback(self, data):
        self.scan_data = data

    def _odom_callback(self, data):
        # Update the robot's current position based on odometry data
        self.robot_position = np.array([data.pose.pose.position.x, data.pose.pose.position.y])

    def step(self, action):
        rospy.loginfo(f"Step called with action: {action}")
        vel_cmd = Twist()
        vel_cmd.linear.x = action[0]
        vel_cmd.angular.z = action[1]
        self.cmd_vel_pub.publish(vel_cmd)

        time.sleep(0.5)
        observation = self.get_observation()
        reward = self._compute_reward(observation, action)
        done = self._is_done(observation)
        info = {
            'min_distance': np.min(observation)
        }

        return observation, reward, done, info

    def reset(self, seed=None):
        # Set the seed if provided
        if seed is not None:
            np.random.seed(seed)

        # Reset the simulation
        self.reset_simulation_service()
        time.sleep(0.5)  # Give the simulation some time to stabilize
        self.wait_for_time_to_progress()

        # Set a new goal position
        self.set_goal_position()

        # Obtain the initial observation
        observation = self.get_observation()
        self.previous_goal_distance = self._compute_goal_distance()
        return observation

    def set_goal_position(self, x=None, y=None):
        # Set a new goal position, either random or specified
        if x is None or y is None:
            self.goal_position = np.array([np.random.uniform(-5.0, 5.0), np.random.uniform(-5.0, 5.0)])
        else:
            self.goal_position = np.array([x, y])

    def wait_for_time_to_progress(self):
        current_time = rospy.Time.now()
        while rospy.Time.now() == current_time:
            time.sleep(0.01)

    def render(self, mode='human'):
        pass

    def close(self):
        rospy.signal_shutdown('Shutting down environment')

    def get_observation(self):
        while self.scan_data is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
        scan_ranges = np.array(self.scan_data.ranges)
        scan_ranges[scan_ranges == np.inf] = self.scan_data.range_max
        reduced_ranges = scan_ranges[::2]  # Reduce to 180 readings for simplicity
        normalized_ranges = reduced_ranges / self.scan_data.range_max
        return normalized_ranges

    def _compute_goal_distance(self):
        # Compute distance from TurtleBot to the goal position
        if self.robot_position is None:
            return float('inf')  # If no position data yet, return a large distance
        return np.linalg.norm(self.goal_position - self.robot_position)

    def _compute_reward(self, observation, action):
        min_distance = np.min(observation)
        collision_threshold = 0.2
        goal_distance = self._compute_goal_distance()

        # Büntetés ütközésért
        if min_distance < collision_threshold:
            reward = -100
        # Jutalom, ha közeledik a célhoz
        elif goal_distance < self.previous_goal_distance:
            reward = 10
        # Büntetés, ha távolodik a célponttól
        else:
            reward = -1
        
        # Előrehaladásért is adunk jutalmat, ha ütközés nem történt
        reward += action[0] * 2.0
        self.previous_goal_distance = goal_distance

        return reward

    def _is_done(self, observation):
        min_distance = np.min(observation)
        collision_threshold = 0.2

        # Done if collision occurs or if the goal is reached
        goal_reached_threshold = 0.5  # If the robot is within 0.5 meters of the goal
        goal_distance = self._compute_goal_distance()

        return min_distance < collision_threshold or goal_distance < goal_reached_threshold
