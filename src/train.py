from custom_env import CustomGazeboEnv
import warnings
warnings.filterwarnings("ignore", category=UserWarning)
# Other necessary imports
import rospy
import gym
from stable_baselines3 import PPO

# Initialize the ROS node
rospy.init_node('gym_gazebo_env', anonymous=True)


env = CustomGazeboEnv()
obs = env.reset()
done = False
while not done:
    obs, reward, done, info = env.step(env.action_space.sample())
    print(obs, reward, done, info)




# Create the environment
env = CustomGazeboEnv()

# Initialize the agent
model = PPO('MlpPolicy', env, verbose=2)

# Train the agent
model.learn(total_timesteps=10000)

# Save the trained model
model.save("ppo_gazebo_model")
