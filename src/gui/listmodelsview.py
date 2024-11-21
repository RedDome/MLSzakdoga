import tkinter as tk
from tkinter import ttk
from utils.listmodels import listModels

def modelSimulateView(root):
    new_window = basicView(root)
    giveSimulateButton(new_window)

def modelSimulateView(root):
    new_window = basicView(root)
    giveResumeButton(new_window)

def basicView(root):
    new_window = tk.Toplevel(root)
    new_window.title("Modellek listája")
    new_window.geometry("600x600")
    new_window.grab_set()

    model_list = ttk.Treeview(new_window, columns=("Iterációszám"))
    model_list.heading("#0", text="Iterációszám")
    model_list.pack()

    listModels(model_list)

    return new_window

def giveSimulateButton(window):
    window.change_method_button = tk.Button(window, text="Modell Szimulálása", command=simulateTraining)
    window.change_method_button.grid(row=1, column=0, pady=5, sticky="s")

def giveResumeButton(window):
    window.change_method_button = tk.Button(window, text="Modell Folytatása", command=continueTraining)
    window.change_method_button.grid(row=1, column=0, pady=5, sticky="s")


def simulateTraining():
    print("Simulating model...")

def continueTraining():
    print("Continuing model training...")