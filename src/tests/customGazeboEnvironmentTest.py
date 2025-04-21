import unittest
import rospy
import time
import warnings
from env.customGazeboEnv import customGazeboEnv
from stable_baselines3.common.env_checker import check_env
import numpy as np
from nav_msgs.msg import Odometry

class customGazeboEnvironmentTest(unittest.TestCase):
    env = None

    def _odom_callback(self, data):
        self.robot_position = np.array([data.pose.pose.position.x, data.pose.pose.position.y], dtype=np.float32)

    @classmethod
    def setUp(cls):
        warnings.simplefilter('ignore', category=ResourceWarning)
        rospy.init_node('gym_gazebo_env', anonymous=True)
        cls.env = customGazeboEnv()

    def test_CheckEnv(self):
        print("test_CheckEnv started!")
        check_env(self.env)

    def test_CheckEnvInitValues(self):
        print("test_CheckEnvStartAndGoalPositions started!")
        self.assertEqual(self.env.robot_position.tolist(), [0, 0])
        self.assertEqual(self.env.goal_position.tolist(), [5, 5])
        self.assertEqual(self.env.robot_orientation, 0)

    def test_ChangeGoalPosition(self):
        print("test_ChangeGoalPositionTest started!")
        new_goal_position = (10, 10)
        self.env.set_goal_position(*new_goal_position)

        self.assertEqual(self.env.goal_position.tolist(), [10, 10])
    
    def test_ResetEnv(self):
        print("test_ResetEnv started!")
        self.assertEqual(self.env.goal_position.tolist(), [5, 5])

        self.env.reset()

        self.assertEqual(self.env.robot_position.tolist(), [0, 0])
        self.assertEqual(self.env.robot_orientation, 0)
        self.assertEqual(self.env.laser_data.tolist(), [0.0])

    def test_Reward(self):
        print("test_Reward started!")
        self.env.goal_position = np.array([0, 0], dtype=np.float32)

        action = 0
        obs, reward, done, _, _ = self.env.step(action)

        self.assertTrue(done)
        self.assertEqual(reward, 100.0)

    def test_Odom(self):
        print("test_Odom started!")
        self.robot_position = [0,0]
        self.odom_sub = rospy.Subscriber('/turtlebot3_burger/odom', Odometry, self._odom_callback)
        
        time.sleep(0.5)

        self.assertNotEqual(self.robot_position[0], 0)
        self.assertNotEqual(self.robot_position[1], 0) 

if __name__ == "__main__":
    unittest.main()