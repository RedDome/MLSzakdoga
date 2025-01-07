import tkinter as tk
from tkinter import messagebox
from gui.changeiterationsview import ChangeIterationsView
from gui.listmodelsview import modelSimulateView, modelContinueView
from utils.train import train

def trainNewModel():
    train()

def simulateModel():
    modelSimulateView(root)
    # giveSimulateButton(root.master)

def continueTraining():
    modelContinueView(root)
    # giveResumeButton(root.master)

def exit():
    root.quit()

def createMainGUI():
    global root
    root = tk.Tk()
    root.title("Gazebo robotszimuláció")
    root.geometry("200x180")
    root.resizable(False, False)

    train_button = tk.Button(root, text="Új Modell tanítása", command=trainNewModel)
    train_button.grid(row=2, column=1, pady=5)

    list_button = tk.Button(root, text="Modellek szimulálása", command=simulateModel)
    list_button.grid(row=3, column=1, pady=5)

    change_method_button = tk.Button(root, text="Tanítás folytatása", command=continueTraining)
    change_method_button.grid(row=4, column=1, pady=5)

    exit_button = tk.Button(root, text="Kilépés", command=exit)
    exit_button.grid(row=6, column=1, pady=5)

    root.mainloop()

