import tkinter as tk
from tkinter import ttk, messagebox

class ListModelsView:
    def __init__(self, master):
        self.new_window = tk.Toplevel(master)
        self.new_window.title("Select Model to render")
        self.new_window.geometry("600x600")
        self.new_window.grab_set()

        self.modelrender_list = ttk.Treeview(self.new_window, columns=("Model Number", "Mean Reward", "Std Reward"))
        self.modelrender_list.heading("#0", text="Model Number")
        self.modelrender_list.heading("#1", text="Mean Reward")
        self.modelrender_list.heading("#2", text="Std Reward")
        self.modelrender_list.pack()

        self.render_button = tk.Button(self.new_window, text="Render Selected Model", command=self.render_selected_model)
        self.render_button.pack()

    def render_selected_model():
        print("")