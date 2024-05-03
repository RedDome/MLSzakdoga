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
        
        self.change_iterations_button = tk.Button(master, text="Change Iterations/Model Learning", command=self.change_iterations)
        self.change_iterations_button.pack()
        
        self.render_model_button = tk.Button(master, text="Render Model", command=self.render_model)
        self.render_model_button.pack()

        self.exit_button = tk.Button(root, text="Exit", command=self.exit)
        self.exit_button.pack()

    def learn(self):
        message = "Learning..."
        self.display_message(message)

        models_dir = "models/PPO"
        logdir = "logs"

        if not os.path.exists(models_dir):
            os.makedirs(models_dir)

        if not os.path.exists(logdir):
            os.makedirs(logdir)

        env = gym.make("LunarLander-v2", render_mode="rgb_array")

        model = PPO("MlpPolicy", env, verbose=1, tensorboard_log=logdir)

        TIMESTEPS = 10000
        itera = length // TIMESTEPS
        for i in range(itera):
            model.learn(total_timesteps=TIMESTEPS, reset_num_timesteps=False, tb_log_name="PPO")
            model.save(f"{models_dir}/{TIMESTEPS*i}")

    def list_models(self):
        message = "Listing models..."
        self.display_message(message)

        models_dir="models/PPO"
        for filename in os.listdir(models_dir):
            if filename.endswith(".zip"):  # Assuming model files are saved as zip files
                model_path = os.path.join(models_dir, filename)
                try:
                    model = PPO.load(model_path)
                    env = gym.make("LunarLander-v2", render_mode="rgb_array")

                    mean_reward, std_reward = evaluate_policy(model, env, n_eval_episodes=2)
                    message = f"Model: {filename}, Mean Reward: {round(mean_reward, 2)}, Std Reward: {round(std_reward, 2)}"
                    self.display_message(message)
                except Exception as e:
                    message = f"Error loading model '{filename}': {e}"
                    self.display_message(message)

    def change_iterations(self):
        message = "Changing iterations/model learning..."
        self.display_message(message)

        self.new_window = tk.Toplevel(self.master)
        self.new_window.title("Change Iterations")
        self.new_window.geometry("400x400")
        self.model_list = ttk.Treeview(self.new_window, columns=("Algorithm Name", "Description"))
        self.model_list.heading("#0", text="Model Name")
        self.model_list.heading("#1", text="Description")
        # elf.model_list.bind('<<ListboxSelect>>', self.update_description)
        # self.model_description = tk.Label(self.new_window, text="")
        # self.model_description.pack()
        self.model_list.pack()
        self.model_list.insert("", tk.END, text = "A2C", values = "test")
        self.model_list.insert("", tk.END, text = "PPO", values = "test")
        self.model_list.insert("", tk.END, text = "DQN", values = "test")
        self.model_list.insert("", tk.END, text = "SAC", values = "test")
        self.model_list.insert("", tk.END, text = "TD3", values = "test")

        self.envi_list = ttk.Treeview(self.new_window, columns=("Environment Name", "Description"))
        self.model_list.heading("#0", text="Environment Name")
        self.model_list.heading("#1", text="Description")
        self.envi_list.pack()
        self.envi_list.insert("", tk.END, text = "LunarLander-v2", values = "test")
        # self.submit_button = tk.Button(self.new_window, text="Submit", command=self.update_iterations)
        # self.submit_button.pack()
        self.new_window.grab_set()

        self.input_field = tk.Entry(self.new_window)
        self.input_field.pack()

        self.submit_button = tk.Button(self.new_window, text="Submit", command=self.update_iterations)
        self.submit_button.pack()

    def update_iterations(self):
        global length

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

        if self.input_field.get():
            length = (int(self.input_field.get()))
        
        message = "Iterations changed..."
        self.display_message(message)

        self.iter_label.config(text=f"Current Iterations: {length}")
        self.model_label.config(text=f"Model Learning Method: {learningmodel}")

        self.new_window.destroy()

    def render_model(self):
        message = "Rendering model..."
        self.display_message(message)

        env = gym.make("LunarLander-v2", render_mode="rgb_array")

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
