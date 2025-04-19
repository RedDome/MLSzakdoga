from env.customGazeboEnv import customGazeboEnv
import warnings
warnings.filterwarnings("ignore", category=UserWarning)

import os
import rospy
from stable_baselines3 import *
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.logger import configure
from utils.sharedValues import sharedValues
from utils.createDirectories import createDirectories
from loguru import logger

def continueTrainingGazebo():
    sv = sharedValues()
    rospy.init_node('gym_gazebo_env', anonymous=True)

    env = customGazeboEnv()
    check_env(env)

    createDirectories()

    obs = env.reset()

    logger.info("xGoal: " + str(sv.xGoal))
    logger.info("yGoal: " + str(sv.yGoal))
    env.set_goal_position(sv.xGoal, sv.yGoal)

    logger.info("length: " + str(sv.length))
    timeSteps = 1000
    itera = sv.length // timeSteps

    logger.info("learningModel: " + str(sv.learningModel))
    if sv.learningModel == "A2C":
        model = A2C.load(sv.modelPath, env=env)
    elif sv.learningModel == "PPO":
        model = PPO.load(sv.modelPath, env=env)
    elif sv.learningModel == "DQN":
        model = DQN.load(sv.modelPath, env=env)
    elif sv.learningModel == "SAC":
        model = SAC.load(sv.modelPath, env=env)
    elif sv.learningModel == "TD3":
        model = TD3.load(sv.modelPath, env=env)
    else:
        logger.error("Unknown learning model! Supported learning models are: A2C, PPO, DQN, SAC, TD3")
        raise ValueError(f"Unknown learning model: {sv.learningModel}")

    new_logger = configure(sv.logFolder, ["stdout", "tensorboard"])
    model.set_logger(new_logger)

    modelFileName= os.path.basename(sv.modelPath)
    savedStep = int(os.path.splitext(modelFileName)[0])

    for i in range(itera):
        model.learn(total_timesteps=timeSteps, reset_num_timesteps=False, tb_log_name=str(sv.learningModel))
        model.save(f"{sv.modelFolder}/{savedStep + timeSteps*(i+1)}")
        logger.info("Model saved at step: " + str(savedStep + timeSteps*(i+1)))

    logger.info("Learning ended!")

    env.close()