import tkinter as tk
import time
import gymnasium as gym

from stable_baselines3 import A2C, PPO
from stable_baselines3.common import base_class
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.callbacks import EvalCallback
import os

class MyApp:
    def __init__(self, master):
        self.master = master
        master.title("ML App")
        

        self.iter_label = tk.Label(master, text ="Current Iterations: 0")
        self.iter_label.pack() 
        
        self.model_label = tk.Label(master, text="Model Learning Method: None")
        self.model_label.pack()

        self.environment_label = tk.Label(master, text="Current Environment: None")
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

    def list_models(self):
        message = "Listing models..."
        self.display_message(message)

    def change_iterations(self):
        message = "Changing iterations/model learning..."
        self.display_message(message)

        self.new_window = tk.Toplevel(self.master)
        self.new_window.title("Change Iterations")
        self.new_window.grab_set()

        self.input_field = tk.Entry(self.new_window)
        self.input_field.pack()

        self.submit_button = tk.Button(self.new_window, text="Submit", command=self.update_iterations)
        self.submit_button.pack()

    def update_iterations(self):
        self.iterations.set(int(self.input_field.get()))
        message = "Iterations changed..."
        self.display_message(message)
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
