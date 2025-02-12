import logging
from env.custom_env import CustomGazeboEnv
import warnings
warnings.filterwarnings("ignore", category=UserWarning)

import rospy
from stable_baselines3 import *
from stable_baselines3.common.env_checker import check_env
from utils.commonvalues import learningModel, length, xGoal, yGoal, logFolder, modelFolder, test
from utils.createDirectories import createDirectories

def train():
    # csak egyszer szükséges meghívni
    rospy.init_node('gym_gazebo_env', anonymous=True)

    createDirectories()

    env = CustomGazeboEnv()
    check_env(env)

    obs = env.reset()

    logging.info("xGoal: " + str(xGoal))
    logging.info("yGoal: " + str(yGoal))
    env.set_goal_position(xGoal, yGoal)

    logging.info("length: " + str(length))
    timeSteps = 1000
    itera = length // timeSteps

    logging.info("learningModel: " + str(learningModel))
    if learningModel == "A2C":
        model = A2C('MlpPolicy', env, verbose=2, n_steps=itera, batch_size=64, ent_coef=0.01, learning_rate=0.0003, tensorboard_log=logFolder)
    elif learningModel == "PPO":
        model = PPO('MlpPolicy', env, verbose=2, n_steps=itera, batch_size=64, ent_coef=0.01, learning_rate=0.0003, tensorboard_log=logFolder)
    elif learningModel == "DQN":
        model = DQN('MlpPolicy', env, verbose=2, n_steps=itera, batch_size=64, ent_coef=0.01, learning_rate=0.0003, tensorboard_log=logFolder)
    elif learningModel == "SAC":
        model = SAC('MlpPolicy', env, verbose=2, n_steps=itera, batch_size=64, ent_coef=0.01, learning_rate=0.0003, tensorboard_log=logFolder)
    elif learningModel == "TD3":
        model = TD3('MlpPolicy', env, verbose=2, n_steps=itera, batch_size=64, ent_coef=0.01, learning_rate=0.0003, tensorboard_log=logFolder)
    else:
        raise ValueError(f"Unknown learning model: {learningModel}")

    logging.info("logFolder: " + str(logFolder))
    logging.info("modelFolder: " + str(modelFolder))
    
    test()

    for i in range(itera):
        model.learn(total_timesteps=timeSteps, reset_num_timesteps=False, tb_log_name=str(learningModel))
        model.save(f"{modelFolder}/{timeSteps*(i+1)}")
        logging.info("Model saved at step: " + str(timeSteps*(i+1)))

    logging.info("Learning ended!")

    env.close()
