# shared values across files
import logging

functionName = ""
length = 10000
learningModel = "PPO"
xGoal = 2.0
yGoal = 1.0
modelPath = ""
tensorboardDataPath = ""

def printvalues():
    logging.info("shared::printvalues Started!")
    logging.info("length: " + str(length))
    logging.info("learningModel: " + learningModel)
    logging.info("x, y goal value: %s, %s", str(xGoal), str(yGoal))
    logging.info("functionName: " + functionName)
    logging.info("modelPath: " + modelPath)
    logging.info("tensorboardDataPath: " + tensorboardDataPath)
    logging.info("shared::printvalues Ended!")

def setLength(newValue):
    global length
    length = newValue

def setFunctionName(newValue):
    global functionName
    functionName = newValue

def setLearningModel(newValue):
    global learningModel
    learningModel = newValue

def setXGoal(newValue):
    global xGoal
    xGoal = newValue

def setYGoal(newValue):
    global yGoal
    yGoal = newValue

def setModelPath(newValue):
    global modelPath
    modelPath = newValue

def setTensorboardDataPath(newValue):
    global tensorboardDataPath
    tensorboardDataPath = newValue