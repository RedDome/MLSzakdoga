from utils.shared import length, learningmodel, printvalues, setLearningModel, setLength
import logging

def update_iterations(self):
        logging.info("Value changing started!")
        selected_model = self.model_list.item(self.model_list.selection())['text']

        logging.info("Learning Model is: " + selected_model)
        if selected_model == "A2C":
            setLearningModel("A2C")
        elif selected_model == "PPO":
            setLearningModel("PPO")
        elif selected_model == "DQN":
            setLearningModel("DQN")
        elif selected_model == "SAC":
            setLearningModel("SAC")
        elif selected_model == "TD3":
            setLearningModel("TD3")
        else:
            raise ValueError(f"Unknown learning model: {learningmodel}")

        logging.info("Selected learningmodel: " + learningmodel)

        if self.input_field.get():
            message = "Please enter a number..."
            logging.info(message)
            setLength(int(self.input_field.get()))
        
        logging.info("New iterations: " + str(length))

        # self.iter_label.config(text=f"Current Iterations: {length}")
        # self.model_label.config(text=f"Model Learning Method: {learningmodel}")
        # self.environment_label.config(text=f"Current Environment: {environment}")

        logging.info("Value changing ended!")

        printvalues()

        self.new_window.destroy()