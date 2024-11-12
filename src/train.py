from custom_env import CustomGazeboEnv
import warnings
warnings.filterwarnings("ignore", category=UserWarning)

import rospy
import gym
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env

# csak egyszer szükséges meghívni
rospy.init_node('gym_gazebo_env', anonymous=True)

env = CustomGazeboEnv()
check_env(env)

obs = env.reset()

env.set_goal_position(x=2.0, y=1.0)

model = PPO('MlpPolicy', env, verbose=2, n_steps=2048, batch_size=64, ent_coef=0.01, learning_rate=0.0003, tensorboard_log='./tb_log')

model.learn(total_timesteps=10000,tb_log_name='vd')

model.save("ppo_gazebo_model")

env.close()
