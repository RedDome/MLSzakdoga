import os
from env.custom_env import CustomGazeboEnv
from stable_baselines3 import A2C, PPO, DQN, SAC, TD3
from stable_baselines3.common.evaluation import evaluate_policy
from utils.shared import learningmodel
import logging
import tkinter as tk
import rospy
import numpy as np

def listModels(self):
    models_dir = "resources/models/"
    for folder in os.listdir(models_dir):
        folder_path = os.path.join(models_dir, folder)
        if os.path.isdir(folder_path):
            for filename in os.listdir(folder_path):
                if filename.endswith(".zip"):  # Assuming model files are saved as zip files
                    model_path = os.path.join(folder_path, filename)
                    logging.info("model found at :" + model_path)
                    self.insert("", tk.END, text = model_path)
            logging.info("Models loaded")

    logging.info("Model Listing ended!")