from env.custom_env import CustomGazeboEnv
import warnings
warnings.filterwarnings("ignore", category=UserWarning)

import rospy
from stable_baselines3 import *
from stable_baselines3.common.env_checker import check_env
from utils.commonvalues import learningModel, length, xGoal, yGoal
from datetime import datetime
import logging
import os

def train():
    # csak egyszer szükséges meghívni
    rospy.init_node('gym_gazebo_env', anonymous=True)

    modelsDirectory = f"resources/models/{learningModel}"
    logDirectory = "resources/logs"

    if not os.path.exists(modelsDirectory):
        os.makedirs(modelsDirectory)

    if not os.path.exists(logDirectory):
        os.makedirs(logDirectory)

    todayDate = datetime.now().strftime("%Y-%m-%d")
    dailyFolder = ""

    suffix = 1
    while True:
        folderName = f"{todayDate}_{str(suffix).zfill(2)}"
        dailyFolder = os.path.join(modelsDirectory, folderName)

        if not os.path.exists(dailyFolder):
            os.makedirs(dailyFolder)
            logging.info("Created Folder: " + dailyFolder)
            break
        else:
            suffix += 1

    env = CustomGazeboEnv()
    check_env(env)

    obs = env.reset()

    env.set_goal_position(xGoal, yGoal)

    timeSteps = 1000
    itera = length // timeSteps

    if learningModel == "A2C":
        model = A2C('MlpPolicy', env, verbose=2, n_steps=itera, batch_size=64, ent_coef=0.01, learning_rate=0.0003, tensorboard_log=logDirectory)
    elif learningModel == "PPO":
        model = PPO('MlpPolicy', env, verbose=2, n_steps=itera, batch_size=64, ent_coef=0.01, learning_rate=0.0003, tensorboard_log=logDirectory)
    elif learningModel == "DQN":
        model = DQN('MlpPolicy', env, verbose=2, n_steps=itera, batch_size=64, ent_coef=0.01, learning_rate=0.0003, tensorboard_log=logDirectory)
    elif learningModel == "SAC":
        model = SAC('MlpPolicy', env, verbose=2, n_steps=itera, batch_size=64, ent_coef=0.01, learning_rate=0.0003, tensorboard_log=logDirectory)
    elif learningModel == "TD3":
        model = TD3('MlpPolicy', env, verbose=2, n_steps=itera, batch_size=64, ent_coef=0.01, learning_rate=0.0003, tensorboard_log=logDirectory)
    else:
        raise ValueError(f"Unknown learning model: {learningModel}")

    
    for i in range(itera):
        model.learn(total_timesteps=timeSteps, reset_num_timesteps=False, tb_log_name=str(learningModel))
        model.save(f"{dailyFolder}/{timeSteps*(i+1)}")
        logging.info("Model saved at step: " + str(timeSteps*(i+1)))

    logging.info("Learning ended!")

    env.close()
