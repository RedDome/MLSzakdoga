import tkinter as tk
from tkinter import messagebox
from gui.changeiterationsview import ChangeIterationsView
from gui.listmodelsview import ListModelsView
from utils.train import train

def train_model():
    train()

def list_models():
    messagebox.showinfo("Info", "List of models displayed.")

def change_training_method():
    messagebox.showinfo("Info", "Training method/iteration updated.")

def openListModelView():
    ListModelsView(root.master)

def openChangeIterationsView():
    ChangeIterationsView(root.master)

def exit_app():
    root.quit()

def create_main_view():
    global root
    root = tk.Tk()
    root.title("Model Trainer")
    root.geometry("520x490")
    root.resizable(False, False)

    iteration_label = tk.Label(root, text="Jelenlegi iterációszám:")
    iteration_label.grid(row=0, column=0, padx=10, pady=10)

    training_method_label = tk.Label(root, text="Kiválasztott tanítási módszer:")
    training_method_label.grid(row=0, column=2, padx=10, pady=10)

    canvas = tk.Canvas(root, width=200, height=200, bg="black")
    canvas.grid(row=1, column=0, columnspan=3, padx=10, pady=10)

    train_button = tk.Button(root, text="Modell tanítása", command=train_model)
    train_button.grid(row=2, column=1, pady=5)

    list_button = tk.Button(root, text="Modellek listázása", command=openListModelView)
    list_button.grid(row=3, column=1, pady=5)

    change_method_button = tk.Button(root, text="Tanítási módszer/iteráció változtatás", command=openChangeIterationsView)
    change_method_button.grid(row=4, column=1, pady=5)

    exit_button = tk.Button(root, text="Kilépés", command=exit_app)
    exit_button.grid(row=6, column=1, pady=5)

    help_button = tk.Button(root, text="Help", command=lambda: messagebox.showinfo("Help", "This is the help section."))
    help_button.grid(row=7, column=0, padx=10, pady=10, sticky="w")

    root.mainloop()

