# shared values across files
from utils.logmessage import log_info

length = 10000
learningmodel = "PPO"
environment = "LunarLander-v2"

def printvalues():
    log_info("shared::printvalues Started!")
    log_info("length: " + str(length))
    log_info("length: " + learningmodel)
    log_info("length: " + environment)
    log_info("shared::printvalues Ended!")

def setLength(newValue):
    global length
    length = newValue

def setLearningModel(newValue):
    global learningmodel
    learningmodel = newValue

def setEnvironment(newValue):
    global environment
    environment = newValue
