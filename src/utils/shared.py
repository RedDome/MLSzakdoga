# shared values across files
import logging

length = 10000
learningmodel = "PPO"
environment = "LunarLander-v2"

def printvalues():
    logging.info("shared::printvalues Started!")
    logging.info("length: " + str(length))
    logging.info("length: " + learningmodel)
    logging.info("length: " + environment)
    logging.info("shared::printvalues Ended!")

def setLength(newValue):
    global length
    length = newValue

def setLearningModel(newValue):
    global learningmodel
    learningmodel = newValue

def setEnvironment(newValue):
    global environment
    environment = newValue
