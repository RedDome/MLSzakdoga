from env.custom_env import CustomGazeboEnv
import warnings
warnings.filterwarnings("ignore", category=UserWarning)

import rospy
from stable_baselines3 import *
from stable_baselines3.common.env_checker import check_env
from utils.commonvalues import learningModel, length, xGoal, yGoal, logFolder, modelFolder
from utils.createDirectories import createDirectories
import utils.commonvalues
import logging

def continueTrainingGazebo():
    rospy.init_node('gym_gazebo_env', anonymous=True)

    env = CustomGazeboEnv()
    check_env(env)

    createDirectories()

    obs = env.reset()

    logging.info("xGoal: " + str(xGoal))
    logging.info("yGoal: " + str(yGoal))
    env.set_goal_position(xGoal, yGoal)

    logging.info("length: " + str(length))
    timeSteps = 1000
    itera = length // timeSteps

    logging.info("learningModel: " + str(learningModel))
    modelPath = utils.commonvalues.modelPath 
    model = PPO.load(modelPath, env=env)

    for i in range(itera):
        model.learn(total_timesteps=timeSteps, reset_num_timesteps=False, tb_log_name=str(learningModel))
        model.save(f"{modelFolder}/{timeSteps*(i+1)}")
        logging.info("Model saved at step: " + str(timeSteps*(i+1)))

    logging.info("Learning ended!")

    env.close()