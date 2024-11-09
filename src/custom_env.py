import gymnasium as gym
from gymnasium import spaces
import numpy as np
import rospy
from std_srvs.srv import Empty
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time

class CustomGazeboEnv(gym.Env):
    def __init__(self):
        super(CustomGazeboEnv, self).__init__()

        # Initialize ROS node
        rospy.init_node('gym_gazebo_env', anonymous=True)

        # Define action and observation space
        self.action_space = spaces.Box(low=np.array([-1.0, -1.0]),
                                       high=np.array([1.0, 1.0]),
                                       dtype=np.float32)

        self.observation_space = spaces.Box(low=0.0,
                                            high=10.0,
                                            shape=(360,),
                                            dtype=np.float32)

        # Publishers and Subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self._scan_callback)
        self.reset_simulation_service = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.unpause_physics_client = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause_physics_client = rospy.ServiceProxy('/gazebo/pause_physics', Empty)

        self.scan_data = None

    def _scan_callback(self, data):
        self.scan_data = data

    def step(self, action):
        rospy.loginfo(f"Step called with action: {action}")
        vel_cmd = Twist()
        vel_cmd.linear.x = action[0]
        vel_cmd.angular.z = action[1]
        self.cmd_vel_pub.publish(vel_cmd)

        time.sleep(0.5)
        observation = self._get_observation()
        reward = self._compute_reward(observation, action)
        done = self._is_done(observation)
        info = {}

        return observation, reward, done, info

    def reset(self, seed=None):
        # Set the seed if provided
        if seed is not None:
            np.random.seed(seed)

        # Reset the simulation
        self.reset_simulation_service()
        time.sleep(0.5)  # Give the simulation some time to stabilize
        self.wait_for_time_to_progress()

        # Obtain the initial observation
        observation = self.get_observation()
        return observation

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
        normalized_ranges = scan_ranges / self.scan_data.range_max
        return normalized_ranges

    def _compute_reward(self, observation, action):
        min_distance = np.min(observation)
        collision_threshold = 0.2
        if min_distance < collision_threshold:
            reward = -100
        else:
            reward = action[0]
        return reward

    def _is_done(self, observation):
        min_distance = np.min(observation)
        collision_threshold = 0.2
        return min_distance < collision_threshold
