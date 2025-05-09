from env.customGazeboEnv import customGazeboEnv
import warnings
warnings.filterwarnings("ignore", category=UserWarning)

import rospy
from stable_baselines3 import *
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.logger import configure
from utils.sharedValues import sharedValues
from utils.createDirectories import createTrainingDirectories
from utils.saveDataFromTensorboardFiles import saveDataFromTensorboardFiles
from loguru import logger

def trainGazebo():
    sv = sharedValues()
    
    rospy.init_node('gym_gazebo_env', anonymous=True)

    createTrainingDirectories()

    env = customGazeboEnv()
    check_env(env)

    obs = env.reset()

    logger.info("xGoal: " + str(sv.xGoal))
    logger.info("yGoal: " + str(sv.yGoal))
    env.set_goal_position(sv.xGoal, sv.yGoal)

    logger.info("length: " + str(sv.length))
    itera = 10
    iteraLength = sv.length // itera
    logLength = iteraLength // 5

    logger.info("learningModel: " + str(sv.learningModel))
    if sv.learningModel == "A2C":
        model = A2C('MlpPolicy', env, verbose=2, n_steps=logLength, batch_size=64, ent_coef=0.01, learning_rate=0.0003, tensorboard_log=sv.logFolder)
    elif sv.learningModel == "PPO":
        model = PPO('MlpPolicy', env, verbose=2, n_steps=logLength, batch_size=64, ent_coef=0.01, learning_rate=0.0003, tensorboard_log=sv.logFolder)
    elif sv.learningModel == "DQN":
        model = DQN('MlpPolicy', env, verbose=2, n_steps=logLength, batch_size=64, ent_coef=0.01, learning_rate=0.0003, tensorboard_log=sv.logFolder)
    elif sv.learningModel == "SAC":
        model = SAC('MlpPolicy', env, verbose=2, n_steps=logLength, batch_size=64, ent_coef=0.01, learning_rate=0.0003, tensorboard_log=sv.logFolder)
    elif sv.learningModel == "TD3":
        model = TD3('MlpPolicy', env, verbose=2, n_steps=logLength, batch_size=64, ent_coef=0.01, learning_rate=0.0003, tensorboard_log=sv.logFolder)
    else:
        logger.error("Unknown learning model! Supported learning models are: A2C, PPO, DQN, SAC, TD3")
        raise ValueError(f"Unknown learning model: {sv.learningModel}")
    
    new_logger = configure(sv.logFolder, ["stdout", "tensorboard"])
    model.set_logger(new_logger)
    
    logger.info("logFolder: " + str(sv.logFolder))
    logger.info("modelFolder: " + str(sv.modelFolder))

    for i in range(itera):
        model.learn(total_timesteps=iteraLength, reset_num_timesteps=False, tb_log_name=str(sv.learningModel))
        model.save(f"{sv.modelFolder}/{iteraLength*(i+1)}")
        logger.info("Model saved at step: " + str(iteraLength*(i+1)))

    logger.info("Learning ended!")

    if sv.saveDataAfterFinished:
        logger.info("Saving data function enabled!")
        sv.setLogFolder(sv.logFolder)
        saveDataFromTensorboardFiles()
        
    env.close()
