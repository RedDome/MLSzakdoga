from utils.shared import length, learningmodel, environment, printvalues, setLearningModel, setEnvironment, setLength
from utils.logmessage import log_info

def update_iterations(self):
        log_info("Value changing started!")
        selected_model = self.model_list.item(self.model_list.selection())['text']

        log_info("Learning Model is: " + selected_model)
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

        log_info("Selected learningmodel: " + learningmodel)

        selected_env = self.envi_list.item(self.envi_list.selection())['text']

        if selected_env == "LunarLander-v2":
            setEnvironment("LunarLander-v2")
        elif selected_env == "CartPole-v1":
            setEnvironment("CartPole-v1")

        log_info("Selected environment: " + environment)

        if self.input_field.get():
            message = "Please enter a number..."
            log_info(message)
            setLength(int(self.input_field.get()))
        
        log_info("New iterations: " + str(length))

        # self.iter_label.config(text=f"Current Iterations: {length}")
        # self.model_label.config(text=f"Model Learning Method: {learningmodel}")
        # self.environment_label.config(text=f"Current Environment: {environment}")

        log_info("Value changing ended!")

        printvalues()

        self.new_window.destroy()