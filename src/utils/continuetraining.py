from env.custom_env import CustomGazeboEnv
import warnings
warnings.filterwarnings("ignore", category=UserWarning)

import rospy
from stable_baselines3 import *
from stable_baselines3.common.env_checker import check_env
from utils.shared import learningmodel, length, xgoal, ygoal
import logging
import os

def continueTrainingGazebo(model_path):
    rospy.init_node('gym_gazebo_env', anonymous=True)

    env = CustomGazeboEnv()
    check_env(env)

    models_dir = f"resources/models/{learningmodel}"
    logdir = "resources/logs"

    if not os.path.exists(models_dir):
        os.makedirs(models_dir)

    if not os.path.exists(logdir):
        os.makedirs(logdir)

    obs = env.reset()

    # env.set_goal_position(x=2.0, y=1.0)
    env.set_goal_position(xgoal, ygoal)

    TIMESTEPS = 10000
    itera = length // TIMESTEPS

    model = PPO.load(model_path, env=env)

    for i in range(itera):
        model.learn(total_timesteps=TIMESTEPS, reset_num_timesteps=False, tb_log_name=str(learningmodel))
        model.save(f"{models_dir}/{TIMESTEPS*(i+1)}")
        logging.info("Model saved at step: " + str(TIMESTEPS*(i+1)))

    logging.info("Learning ended!")

    # model.save("")

    env.close()