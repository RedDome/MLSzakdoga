import gym
from gym import spaces
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
        # Actions: [linear_velocity, angular_velocity]
        self.action_space = spaces.Box(low=np.array([-1.0, -1.0]),
                                       high=np.array([1.0, 1.0]),
                                       dtype=np.float32)

        # Observations: Laser scan data (e.g., 360-degree lidar)
        # For simplicity, we use a fixed-size observation
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
        # Create Twist message from action
        vel_cmd = Twist()
        vel_cmd.linear.x = action[0]
        vel_cmd.angular.z = action[1]

        # Publish velocity command
        self.cmd_vel_pub.publish(vel_cmd)

        # Wait for a time step
        #rospy.sleep(0.1)
        time.sleep(0.5)


        # Get observation
        observation = self._get_observation()

        # Calculate reward
        reward = self._compute_reward(observation, action)

        # Check if episode is done
        done = self._is_done(observation)

        # Additional info (optional)
        info = {}

        return observation, reward, done, info

    def reset(self):
        # Reseteli a szimulációt
        self.reset_simulation_service()

        # Várakozás a szimuláció stabilizálódására
        time.sleep(0.5)  # Használja a time.sleep()-et

        # Esetleg ellenőrizze, hogy az idő előre halad
        self.wait_for_time_to_progress()

        # További inicializálási lépések
        observation = self.get_observation()
        return observation

    def wait_for_time_to_progress(self):
        current_time = rospy.Time.now()
        while rospy.Time.now() == current_time:
            time.sleep(0.01)

    def get_observation(self):
        # Adj néhány másodpercet a szimuláció inicializálódásához
        time.sleep(5)  # vagy próbálj akár nagyobb értéket, ha szükséges
        # Megszerzi a jelenlegi megfigyelést az érzékelőkből vagy a szimulációból
        laser_scan = rospy.wait_for_message('/scan', LaserScan)
        # Feldolgozza az érzékelő adatokat
        observation = self.process_laser_scan(laser_scan)
        return observation

    def process_laser_scan(self, laser_scan):
        # Az érzékelő adatok feldolgozása az ügynök számára megfelelő formába
        ranges = np.array(laser_scan.ranges)
        ranges = np.clip(ranges, laser_scan.range_min, laser_scan.range_max)
        return ranges

    def render(self, mode='human'):
        # Rendering is handled by Gazebo
        pass

    def close(self):
        rospy.signal_shutdown('Shutting down environment')

    def _get_observation(self):
        # Wait until scan data is available
        while self.scan_data is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

        # Process scan data
        scan_ranges = np.array(self.scan_data.ranges)
        # Replace 'inf' values with max range
        scan_ranges[scan_ranges == np.inf] = self.scan_data.range_max
        # Normalize the scan data
        normalized_ranges = scan_ranges / self.scan_data.range_max

        return normalized_ranges

    def _compute_reward(self, observation, action):
        # Example reward function: Encourage forward motion and penalize collisions
        min_distance = np.min(observation)
        collision_threshold = 0.2  # Threshold distance to consider collision

        if min_distance < collision_threshold:
            reward = -100  # Collision penalty
        else:
            reward = action[0]  # Reward forward motion

        return reward

    def _is_done(self, observation):
        # Episode is done if robot collides
        min_distance = np.min(observation)
        collision_threshold = 0.2

        return min_distance < collision_threshold

