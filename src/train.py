from custom_env import CustomGazeboEnv
import warnings
warnings.filterwarnings("ignore", category=UserWarning)

# Egyéb szükséges importok
import rospy
import gym
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env

# Initialize the ROS node (csak egyszer szükséges meghívni)
rospy.init_node('gym_gazebo_env', anonymous=True)

# Create the environment
env = CustomGazeboEnv()

# Set a custom goal position
env.set_goal_position(x=3.0, y=3.0)

# It will check your custom environment and output additional warnings if needed
check_env(env)

# Reset the environment before training
obs = env.reset()

# Initialize the agent with fine-tuned parameters
model = PPO('MlpPolicy', env, verbose=2, n_steps=2048, batch_size=64, ent_coef=0.01, learning_rate=0.0003)

# Train the agent
model.learn(total_timesteps=10000)

# Save the trained model
model.save("ppo_gazebo_model")

# Close the environment after training
env.close()
