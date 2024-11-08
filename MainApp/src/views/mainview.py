import tkinter as tk
from tkinter import messagebox
from .changeiterationsview import ChangeIterationsView
from .listmodelsview import ListModelsView

def train_model():
    messagebox.showinfo("Info", "Model training started.")

def list_models():
    messagebox.showinfo("Info", "List of models displayed.")

def change_training_method():
    messagebox.showinfo("Info", "Training method/iteration updated.")

def display_model():
    messagebox.showinfo("Info", "Model displayed.")

def openListModelView():
    ListModelsView(root.master)

def openChangeIterationsView():
    ChangeIterationsView(root.master)

def exit_app():
    root.quit()

# Main application window
root = tk.Tk()
root.title("Model Trainer")
root.geometry("520x490")
root.resizable(False, False)

# Labels for iteration and training method
iteration_label = tk.Label(root, text="Jelenlegi iterációszám:")
iteration_label.grid(row=0, column=0, padx=10, pady=10)

training_method_label = tk.Label(root, text="Kiválasztott tanítási módszer:")
training_method_label.grid(row=0, column=2, padx=10, pady=10)

# Placeholder for central area (could be an image or other content)
canvas = tk.Canvas(root, width=200, height=200, bg="black")
canvas.grid(row=1, column=0, columnspan=3, padx=10, pady=10)

# Buttons
train_button = tk.Button(root, text="Modell tanítása", command=train_model)
train_button.grid(row=2, column=1, pady=5)

list_button = tk.Button(root, text="Modellek listázása", command=openListModelView)
list_button.grid(row=3, column=1, pady=5)

change_method_button = tk.Button(root, text="Tanítási módszer/iteráció változtatás", command=openChangeIterationsView)
change_method_button.grid(row=4, column=1, pady=5)

display_model_button = tk.Button(root, text="Modell megjelenítése", command=display_model)
display_model_button.grid(row=5, column=1, pady=5)

exit_button = tk.Button(root, text="Kilépés", command=exit_app)
exit_button.grid(row=6, column=1, pady=5)

# Help button at the bottom left
help_button = tk.Button(root, text="Help", command=lambda: messagebox.showinfo("Help", "This is the help section."))
help_button.grid(row=7, column=0, padx=10, pady=10, sticky="w")

root.mainloop()

