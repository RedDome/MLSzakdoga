import tkinter as tk
import time
import gymnasium as gym
from stable_baselines3 import A2C, PPO, DQN, SAC, TD3
from stable_baselines3.common import base_class
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.callbacks import EvalCallback
import os
from tkinter import ttk

length = 100000
learningmodel = "PPO"
environment = "LunarLander-v2"


class MyApp:
    def __init__(self, master):
        self.master = master
        master.title("ML App")
        
        self.iter_label = tk.Label(master, text=f"Current Iterations: {length}")
        self.iter_label.pack() 
        
        self.model_label = tk.Label(master, text=f"Model Learning Method: {learningmodel}")
        self.model_label.pack()

        self.environment_label = tk.Label(master, text=f"Current Environment: {environment}")
        self.environment_label.pack()
  
        self.display = tk.Text(master, height=10, width=40)
        self.display.pack()
        
        self.learn_button = tk.Button(master, text="Learn", command=self.learn)
        self.learn_button.pack()
        
        self.list_models_button = tk.Button(master, text="List Models", command=self.list_models)
        self.list_models_button.pack()
        
        self.change_iterations_button = tk.Button(master, text="Change Iterations/Algorithm/Environment", command=self.change_iterations)
        self.change_iterations_button.pack()
        
        self.render_model_button = tk.Button(master, text="Render Model", command=self.render_model)
        self.render_model_button.pack()

        self.exit_button = tk.Button(root, text="Exit", command=self.exit)
        self.exit_button.pack()

    def learn(self):
        message = "Learning..."
        self.display_message(message)

        models_dir = f"models/{learningmodel}"
        logdir = "logs"

        if not os.path.exists(models_dir):
            os.makedirs(models_dir)

        if not os.path.exists(logdir):
            os.makedirs(logdir)

        env = gym.make(str(environment), render_mode="rgb_array")

        match learningmodel:
            case "A2C":
                model = A2C("MlpPolicy", env, verbose=1, tensorboard_log=logdir)
            case "PPO":
                model = PPO("MlpPolicy", env, verbose=1, tensorboard_log=logdir)
            case "DQN":
                model = DQN("MlpPolicy", env, verbose=1, tensorboard_log=logdir)
            case "SAC":
                model = SAC("MlpPolicy", env, verbose=1, tensorboard_log=logdir)
            case "TD3":
                model = TD3("MlpPolicy", env, verbose=1, tensorboard_log=logdir)

        TIMESTEPS = 10000
        itera = length // TIMESTEPS
        for i in range(itera):
            model.learn(total_timesteps=TIMESTEPS, reset_num_timesteps=False, tb_log_name="PPO")
            model.save(f"{models_dir}/{TIMESTEPS*i}")

    def list_models(self):
        message = "Listing models..."
        self.display_message(message)

        # models_dir=f"models/{learningmodel}"
        
        models_dir = "models/"
        for folder in os.listdir(models_dir):
            folder_path = os.path.join(models_dir, folder)
            if os.path.isdir(folder_path):
                for filename in os.listdir(folder_path):
                    if filename.endswith(".zip"):  # Assuming model files are saved as zip files
                        model_path = os.path.join(folder_path, filename)
                        try:
                            match learningmodel:
                                case "A2C":
                                    model = A2C.load(model_path)
                                case "PPO":
                                    model = PPO.load(model_path)
                                case "DQN":
                                    model = DQN.load(model_path)
                                case "SAC":
                                    model = SAC.load(model_path)
                                case "TD3":
                                    model = TD3.load(model_path)
                    
                            env = gym.make(str(environment), render_mode="rgb_array")

                            mean_reward, std_reward = evaluate_policy(model, env, n_eval_episodes=2)
                            message = f"Model full path: {model_path}, Mean Reward: {round(mean_reward, 2)}, Std Reward: {round(std_reward, 2)}"
                            self.display_message(message)
                        except Exception as e:
                            message = f"Error loading model '{filename}': {e}"
                            self.display_message(message)

    def change_iterations(self):
        message = "Changing iterations/model learning..."
        self.display_message(message)

        self.new_window = tk.Toplevel(self.master)
        self.new_window.title("Change Iterations")
        self.new_window.geometry("600x600")
        self.new_window.grab_set()

        self.model_list = ttk.Treeview(self.new_window, columns=("Algorithm Name", "Description"))
        self.model_list.heading("#0", text="Model Name")
        self.model_list.heading("#1", text="Description")
        self.model_list.pack()
        self.model_list.insert("", tk.END, text = "A2C", values = "test")
        self.model_list.insert("", tk.END, text = "PPO", values = "test")
        self.model_list.insert("", tk.END, text = "DQN", values = "test")
        self.model_list.insert("", tk.END, text = "SAC", values = "test")
        self.model_list.insert("", tk.END, text = "TD3", values = "test")

        self.envi_list = ttk.Treeview(self.new_window, columns=("Environment Name", "Description"))
        self.envi_list.heading("#0", text="Environment Name")
        self.envi_list.heading("#1", text="Description")
        self.envi_list.pack()
        self.envi_list.insert("", tk.END, text = "LunarLander-v2", values = "test")
        self.envi_list.insert("", tk.END, text = "CartPole-v1", values = "test")

        self.change_iter_label = tk.Label(self.new_window, text=f"Current Iterations: {length}, change below:")
        self.change_iter_label.pack()
        self.input_field = tk.Entry(self.new_window)
        self.input_field.pack()

        self.submit_button = tk.Button(self.new_window, text="Save Changes", command=self.update_iterations)
        self.submit_button.pack()

    def update_iterations(self):
        global length
        global learningmodel
        global environment

        selected_model = self.model_list.item(self.model_list.selection())['text']
        
        match selected_model:
            case "A2C":
                learningmodel = "A2C"
            case "PPO":
                learningmodel = "PPO"
            case "DQN":
                learningmodel = "DQN"
            case "SAC":
                learningmodel = "SAC"
            case "TD3":
                learningmodel = "TD3"

        selected_env = self.envi_list.item(self.envi_list.selection())['text']

        match selected_env:
            case "LunarLander-v2":
                environment = "LunarLander-v2"
            case "CartPole-v1":
                environment = "CartPole-v1"

        if self.input_field.get():
            message = "Please enter a number..."
            self.display_message(message)
            length = (int(self.input_field.get()))
        
        message = "Iterations changed..."
        self.display_message(message)

        self.iter_label.config(text=f"Current Iterations: {length}")
        self.model_label.config(text=f"Model Learning Method: {learningmodel}")
        self.environment_label.config(text=f"Current Environment: {environment}")

        self.new_window.destroy()

    def render_model(self):
        message = "Rendering model..."
        self.display_message(message)

        self.new_window = tk.Toplevel(self.master)
        self.new_window.title("Select Model to render")
        self.new_window.geometry("600x600")
        self.new_window.grab_set()

        self.modelrender_list = ttk.Treeview(self.new_window, columns=("Model Number", "Mean Reward", "Std Reward"))
        self.modelrender_list.heading("#0", text="Model Number")
        self.modelrender_list.heading("#1", text="Mean Reward")
        self.modelrender_list.heading("#2", text="Std Reward")
        self.modelrender_list.pack()

        models_dir="testmodel"
        for filename in os.listdir(models_dir):
            if filename.endswith(".zip"):  # Assuming model files are saved as zip files
                model_path = os.path.join(models_dir, filename)
                try:
                    model = PPO.load(model_path)
                    env = gym.make({environment}, render_mode="rgb_array")

                    mean_reward, std_reward = evaluate_policy(model, env, n_eval_episodes=2)
                    self.modelrender_list.insert("", tk.END, text = filename, values = (round(mean_reward, 2), round(std_reward, 2)))
                except Exception as e:
                    message = f"Error loading model '{filename}': {e}"
                    self.display_message(message)

        self.render_button = tk.Button(self.new_window, text="Render Selected Model", command=self.render_selected_model)
        self.render_button.pack()

    def render_selected_model(self):
        env = gym.make(str(environment), render_mode="rgb_array")

        model_path = f"testmodel/280000"
        model = PPO.load(model_path, env=env)

        mean_reward, std_reward = evaluate_policy(model, model.get_env(), n_eval_episodes=10)

        vec_env = model.get_env()
        obs = vec_env.reset()
        for i in range(1000):
            action, _states = model.predict(obs, deterministic=True)
            obs, rewards, dones, info = vec_env.step(action)
            vec_env.render("human")
            
        vec_env.close()

        self.new_window.destroy()

    def exit(self):
        message = "Exiting..."
        print(message)
        self.display_message(message)
        self.master.destroy()  # Destroy the main window

    def display_message(self, message):
        self.display.insert(tk.END, message + "\n")
        self.display.see(tk.END)

root = tk.Tk()
app = MyApp(root)
root.mainloop()
