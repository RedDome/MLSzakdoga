from env.custom_env import CustomGazeboEnv
import warnings
warnings.filterwarnings("ignore", category=UserWarning)

import rospy
from stable_baselines3 import *
from stable_baselines3.common.env_checker import check_env
from utils.commonvalues import learningModel, length, xGoal, yGoal
import utils.commonvalues
from datetime import datetime
import logging
import os

def continueTrainingGazebo():
    rospy.init_node('gym_gazebo_env', anonymous=True)

    env = CustomGazeboEnv()
    check_env(env)

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

    obs = env.reset()

    env.set_goal_position(xGoal, yGoal)

    timeSteps = 10000
    itera = length // timeSteps

    modelPath = utils.commonvalues.modelPath 
    model = PPO.load(modelPath, env=env)

    for i in range(itera):
        model.learn(total_timesteps=timeSteps, reset_num_timesteps=False, tb_log_name=str(learningModel))
        model.save(f"{modelsDirectory}/{timeSteps*(i+1)}")
        logging.info("Model saved at step: " + str(timeSteps*(i+1)))

    logging.info("Learning ended!")

    # model.save("")

    env.close()