from env.custom_env import CustomGazeboEnv
import warnings
warnings.filterwarnings("ignore", category=UserWarning)

import rospy
from stable_baselines3 import *
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.logger import configure
import utils.commonvalues as cm
from utils.createDirectories import createDirectories
from utils.saveDataFromTensorboardFiles import saveDataFromTensorboardFiles
from loguru import logger

def train():
    # csak egyszer szükséges meghívni
    rospy.init_node('gym_gazebo_env', anonymous=True)

    createDirectories()

    env = CustomGazeboEnv()
    check_env(env)

    obs = env.reset()

    logger.info("xGoal: " + str(cm.xGoal))
    logger.info("yGoal: " + str(cm.yGoal))
    env.set_goal_position(cm.xGoal, cm.yGoal)

    logger.info("length: " + str(cm.length))
    timeSteps = 1000
    itera = cm.length // timeSteps

    logger.info("learningModel: " + str(cm.learningModel))
    if cm.learningModel == "A2C":
        model = A2C('MlpPolicy', env, verbose=2, n_steps=itera, batch_size=64, ent_coef=0.01, learning_rate=0.0003, tensorboard_log=cm.logFolder)
    elif cm.learningModel == "PPO":
        model = PPO('MlpPolicy', env, verbose=2, n_steps=itera, batch_size=64, ent_coef=0.01, learning_rate=0.0003, tensorboard_log=cm.logFolder)
    elif cm.learningModel == "DQN":
        model = DQN('MlpPolicy', env, verbose=2, n_steps=itera, batch_size=64, ent_coef=0.01, learning_rate=0.0003, tensorboard_log=cm.logFolder)
    elif cm.learningModel == "SAC":
        model = SAC('MlpPolicy', env, verbose=2, n_steps=itera, batch_size=64, ent_coef=0.01, learning_rate=0.0003, tensorboard_log=cm.logFolder)
    elif cm.learningModel == "TD3":
        model = TD3('MlpPolicy', env, verbose=2, n_steps=itera, batch_size=64, ent_coef=0.01, learning_rate=0.0003, tensorboard_log=cm.logFolder)
    else:
        raise ValueError(f"Unknown learning model: {cm.learningModel}")
    
    new_logger = configure(cm.logFolder, ["stdout", "tensorboard"])
    model.set_logger(new_logger)
    
    logger.info("logFolder: " + str(cm.logFolder))
    logger.info("modelFolder: " + str(cm.modelFolder))

    for i in range(itera):
        model.learn(total_timesteps=timeSteps, reset_num_timesteps=False, tb_log_name=str(cm.learningModel))
        model.save(f"{cm.modelFolder}/{timeSteps*(i+1)}")
        logger.info("Model saved at step: " + str(timeSteps*(i+1)))

    logger.info("Learning ended!")

    if cm.saveDataAfterFinished:
        logger.info("Saving data function enabled!")
        cm.setLogFolder(f"{cm.logFolder}/{cm.learningModel}_0")
        saveDataFromTensorboardFiles()
        
    env.close()
