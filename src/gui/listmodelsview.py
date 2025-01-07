import tkinter as tk
from tkinter import ttk
from utils.listmodels import listModels
import time
import logging
from utils.continuetraining import continueTrainingGazebo

def modelSimulateView(root):
    new_window = basicView(root)
    giveSimulateButton(new_window)

def modelContinueView(root):
    new_window = basicView(root)
    giveContinueButton(new_window)

def basicView(root):
    new_window = tk.Toplevel(root)
    new_window.title("Modellek listája")
    new_window.geometry("600x600")
    new_window.grab_set()

    model_list = ttk.Treeview(new_window, columns=("Iterációszám"))
    model_list.heading("#0", text="Iterációszám")
    model_list.heading("#1", text="Elérhetőség")
    model_list.pack(expand=True, fill=tk.BOTH)

    # root.withdraw()

    # root.deiconify()

    listModels(model_list)

    new_window.model_list = model_list

    return new_window

def giveSimulateButton(window):
    window.change_method_button = tk.Button(window, text="Modell Szimulálása", command=lambda: simulateTraining(window))
    window.change_method_button.pack(pady=5)

def giveContinueButton(window):
    window.change_method_button = tk.Button(window, text="Modell Folytatása", command=lambda: continueTraining(window))
    window.change_method_button.pack(pady=5)


def simulateTraining(window):
    selected_item = window.model_list.item(window.model_list.selection())
    model_number = selected_item['text']
    model_path = selected_item['values'][0]
    full_path = f"{model_path}/{model_number}"
    logging.info("model_path " + full_path)
    logging.info("Simulating model...")
    continueTrainingGazebo(full_path)

def continueTraining(window):
    model_number = window.model_list.item(window.model_list.selection())['text']
    model_path = window.model_list.item(window.model_list.selection())['values']
    logging.info("model_path %s %s", model_number, model_path)
    logging.info("Continuing model training...")