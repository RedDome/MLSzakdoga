import tkinter as tk
from tkinter import ttk
from utils.listmodels import listModels

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

        listModels(self)