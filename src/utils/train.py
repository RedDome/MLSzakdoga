from env.custom_env import CustomGazeboEnv
import warnings
warnings.filterwarnings("ignore", category=UserWarning)

import rospy
from stable_baselines3 import *
from stable_baselines3.common.env_checker import check_env
from utils.shared import learningmodel, length, xgoal, ygoal
import logging
import os

def train():
    # csak egyszer szükséges meghívni
    rospy.init_node('gym_gazebo_env', anonymous=True)

    models_dir = f"resources/models/{learningmodel}"
    logdir = "resources/logs"

    if not os.path.exists(models_dir):
        os.makedirs(models_dir)

    if not os.path.exists(logdir):
        os.makedirs(logdir)

    env = CustomGazeboEnv()
    check_env(env)

    obs = env.reset()

    # env.set_goal_position(x=2.0, y=1.0)
    env.set_goal_position(xgoal, ygoal)

    TIMESTEPS = 1000
    itera = length // TIMESTEPS

    if learningmodel == "A2C":
        model = A2C('MlpPolicy', env, verbose=2, n_steps=itera, batch_size=64, ent_coef=0.01, learning_rate=0.0003, tensorboard_log=logdir)
    elif learningmodel == "PPO":
        model = PPO('MlpPolicy', env, verbose=2, n_steps=itera, batch_size=64, ent_coef=0.01, learning_rate=0.0003, tensorboard_log=logdir)
    elif learningmodel == "DQN":
        model = DQN('MlpPolicy', env, verbose=2, n_steps=itera, batch_size=64, ent_coef=0.01, learning_rate=0.0003, tensorboard_log=logdir)
    elif learningmodel == "SAC":
        model = SAC('MlpPolicy', env, verbose=2, n_steps=itera, batch_size=64, ent_coef=0.01, learning_rate=0.0003, tensorboard_log=logdir)
    elif learningmodel == "TD3":
        model = TD3('MlpPolicy', env, verbose=2, n_steps=itera, batch_size=64, ent_coef=0.01, learning_rate=0.0003, tensorboard_log=logdir)
    else:
        raise ValueError(f"Unknown learning model: {learningmodel}")

    
    for i in range(itera):
        model.learn(total_timesteps=TIMESTEPS, reset_num_timesteps=False, tb_log_name=str(learningmodel))
        model.save(f"{models_dir}/{TIMESTEPS*(i+1)}")
        logging.info("Model saved at step: " + str(TIMESTEPS*(i+1)))

    logging.info("Learning ended!")

    model.save("ppo_gazebo_model")

    env.close()
