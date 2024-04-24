import gymnasium as gym

from stable_baselines3 import A2C, PPO
from stable_baselines3.common import base_class
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.callbacks import EvalCallback
import os

length = 10000

def learn():
    models_dir = "models/PPO"
    logdir = "logs"

    if not os.path.exists(models_dir):
        os.makedirs(models_dir)

    if not os.path.exists(logdir):
        os.makedirs(logdir)

    # Create environment
    env = gym.make("LunarLander-v2", render_mode="rgb_array")

    #Instantiate the agent
    model = PPO("MlpPolicy", env, verbose=1, tensorboard_log=logdir)

    # Create callback
    # callback = EvalCallback(env, best_model_save_path=models_dir,
    #                         log_path=logdir, eval_freq=500,
    #                         deterministic=True, render=False)

    TIMESTEPS = 10000
    itera = length // TIMESTEPS
    for i in range(itera):
        model.learn(total_timesteps=TIMESTEPS, reset_num_timesteps=False, tb_log_name="PPO")
        # mean_reward, std_reward = evaluate_policy(model, model.get_env(), n_eval_episodes=10)
        model.save(f"{models_dir}/{TIMESTEPS*i}")

def listModels():
    models_dir="models/PPO"
    for filename in os.listdir(models_dir):
        if filename.endswith(".zip"):  # Assuming model files are saved as zip files
            model_path = os.path.join(models_dir, filename)
            try:
                model = PPO.load(model_path)
                env = gym.make("LunarLander-v2", render_mode="rgb_array")

                mean_reward, std_reward = evaluate_policy(model, env, n_eval_episodes=2)
                # mean_reward = model.mean_reward if hasattr(model, "mean_reward") else "N/A"
                # std_reward = model.std_reward if hasattr(model, "std_reward") else "N/A"
                print(f"Model: {filename}, Mean Reward: {mean_reward}, Std Reward: {std_reward}")
            except Exception as e:
                print(f"Error loading model '{filename}': {e}")


def changeIterations():
    global length
    print("Current Iterations ", length)

    newnum = input("Enter Iterations: ")

    length = int(newnum)

    print("Iterations changed to ", length)

def renderModel():
    newnum = input("Which model do you want to load: ")

    env = gym.make("LunarLander-v2", render_mode="rgb_array")

    model_path = f"models/PPO/{newnum}"
    model = PPO.load(model_path, env=env)

    mean_reward, std_reward = evaluate_policy(model, model.get_env(), n_eval_episodes=10)

    vec_env = model.get_env()
    obs = vec_env.reset()
    for i in range(1000):
        action, _states = model.predict(obs, deterministic=True)
        obs, rewards, dones, info = vec_env.step(action)
        vec_env.render("human")

def main():
    while True:
        print("\nMenu:")
        print("1. Learn a model")
        print("2. List models")
        print("3. Change iterations")
        print("4. Render a model")
        print("5. Exit")
        
        choice = input("Enter your choice: ")

        if choice == '1':           
            learn()    
        elif choice == '2':
            listModels()
        elif choice == '3':
            changeIterations()
        elif choice == '4':
            renderModel()
        elif choice == '5':
            print("Exiting...")
            break
        else:
            print("Invalid choice. Please enter a number between 1 and 5.")

if __name__ == "__main__":
    main()
