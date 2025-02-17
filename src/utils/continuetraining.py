from env.custom_env import CustomGazeboEnv
import warnings
warnings.filterwarnings("ignore", category=UserWarning)

import rospy
from stable_baselines3 import *
from stable_baselines3.common.env_checker import check_env
import utils.commonvalues as cm
from utils.createDirectories import createDirectories
from loguru import logger

def continueTrainingGazebo():
    rospy.init_node('gym_gazebo_env', anonymous=True)

    env = CustomGazeboEnv()
    check_env(env)

    createDirectories()

    obs = env.reset()

    logger.info("xGoal: " + str(cm.xGoal))
    logger.info("yGoal: " + str(cm.yGoal))
    env.set_goal_position(cm.xGoal, cm.yGoal)

    logger.info("length: " + str(cm.length))
    timeSteps = 1000
    itera = cm.length // timeSteps

    logger.info("learningModel: " + str(cm.learningModel))
    model = PPO.load(cm.modelPath, env=env)

    for i in range(itera):
        model.learn(total_timesteps=timeSteps, reset_num_timesteps=False, tb_log_name=str(cm.learningModel))
        model.save(f"{cm.modelFolder}/{timeSteps*(i+1)}")
        logger.info("Model saved at step: " + str(timeSteps*(i+1)))

    logger.info("Learning ended!")

    env.close()