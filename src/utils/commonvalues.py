# shared values across files
from loguru import logger

functionName = ""
length = 10000
learningModel = "PPO"
xGoal = 2.0
yGoal = 1.0
modelPath = ""
csvFilePath = ""
saveDataAfterFinished = False
logFolder = ""
modelFolder = ""

def printvalues():
    logger.info("shared::printvalues Started!")
    logger.info("length: " + str(length))
    logger.info("learningModel: " + learningModel)
    logger.info("x, y goal value: %s, %s", str(xGoal), str(yGoal))
    logger.info("functionName: " + functionName)
    logger.info("modelPath: " + modelPath)
    logger.info("shared::printvalues Ended!")

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