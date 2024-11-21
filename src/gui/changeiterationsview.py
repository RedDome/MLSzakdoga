import tkinter as tk
from tkinter import ttk
from utils.changevalues import update_iterations

class ChangeIterationsView:
    def __init__(self, master, ):
        self.master = master
        self.current_iterations = 10
        self.new_window = tk.Toplevel(self.master)
        self.new_window.title("Change Iterations")
        self.new_window.geometry("600x600")
        self.new_window.grab_set()

        self.model_list = ttk.Treeview(self.new_window, columns=("Algorithm Name", "Description"))
        self.model_list.heading("#0", text="Model Name")
        self.model_list.heading("#1", text="Description")
        self.model_list.pack()
        self.model_list.insert("", tk.END, text="A2C", values="test")
        self.model_list.insert("", tk.END, text="PPO", values="test")
        self.model_list.insert("", tk.END, text="DQN", values="test")
        self.model_list.insert("", tk.END, text="SAC", values="test")
        self.model_list.insert("", tk.END, text="TD3", values="test")

        self.change_iter_label = tk.Label(self.new_window, text=f"Current Iterations: {self.current_iterations}, change below:")
        self.change_iter_label.pack()
        self.input_field = tk.Entry(self.new_window)
        self.input_field.pack()

        self.submit_button = tk.Button(self.new_window, text="Save Changes", command=self.update)
        self.submit_button.pack()

    def update(self):
        update_iterations(self)