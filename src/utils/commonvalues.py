# shared values across files
import logging

functionName = ""
length = 10000
learningModel = "PPO"
xGoal = 2.0
yGoal = 1.0
modelPath = ""
tensorboardDataPath = ""
csvFilePath = ""
saveDataAfterFinished = False
logFolder = ""
modelFolder = ""

def printvalues():
    logging.info("shared::printvalues Started!")
    logging.info("length: " + str(length))
    logging.info("learningModel: " + learningModel)
    logging.info("x, y goal value: %s, %s", str(xGoal), str(yGoal))
    logging.info("functionName: " + functionName)
    logging.info("modelPath: " + modelPath)
    logging.info("tensorboardDataPath: " + tensorboardDataPath)
    logging.info("shared::printvalues Ended!")

def test():
    logging.info("logFolder: " + str(logFolder))
    logging.info("modelFolder: " + str(modelFolder))

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

def setCSVFilePath(newValue):
    global csvFilePath
    csvFilePath = newValue

def setSaveDataAfterFinished(newValue):
    global saveDataAfterFinished
    saveDataAfterFinished = newValue

def setLogFolder(newValue):
    global logFolder
    logFolder = newValue

def setModelFolder(newValue):
    global modelFolder
    modelFolder = newValue