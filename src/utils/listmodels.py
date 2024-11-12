import os
import gymnasium as gym
from custom_env import CustomGazeboEnv
from stable_baselines3 import A2C, PPO, DQN, SAC, TD3
from stable_baselines3.common import base_class
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.callbacks import EvalCallback
from utils.shared import length, learningmodel, environment
from utils.logmessage import log_info, log_error
import tkinter as tk
import rospy

def listModels(self):
    log_info("Model Listing started!")
    rospy.init_node('gym_gazebo_env', anonymous=True)

    models_dir = "resources/models/"
    for folder in os.listdir(models_dir):
        folder_path = os.path.join(models_dir, folder)
        if os.path.isdir(folder_path):
            for filename in os.listdir(folder_path):
                if filename.endswith(".zip"):  # Assuming model files are saved as zip files
                    model_path = os.path.join(folder_path, filename)
                    log_info("model found at :" + model_path)
                    log_info("Learning Model is: " + learningmodel)
                    if learningmodel == "A2C":
                        model = A2C.load(model_path)
                    elif learningmodel == "PPO":
                        model = PPO.load(model_path)
                    elif learningmodel == "DQN":
                        model = DQN.load(model_path)
                    elif learningmodel == "SAC":
                        model = SAC.load(model_path)
                    elif learningmodel == "TD3":
                        model = TD3.load(model_path)
                    else:
                        raise ValueError(f"Unknown learning model: {learningmodel}")
                    
                    try:
                        # env = gym.make(str(environment), render_mode="rgb_array")
                        env = CustomGazeboEnv()

                        mean_reward, std_reward = evaluate_policy(model, env, n_eval_episodes=1)
                        self.modelrender_list.insert("", tk.END, text = model_path, values = (round(mean_reward, 2), round(std_reward, 2)))
                    except Exception as e:
                        message = f"Error loading model '{filename}': {e}"
                        log_error(message)
            log_info("Models loaded")

    log_info("Model Listing ended!")