# shared values across files
import logging

functionname = ""
length = 10000
learningmodel = "PPO"
xgoal = 2.0
ygoal = 1.0

def printvalues():
    logging.info("shared::printvalues Started!")
    logging.info("length: " + str(length))
    logging.info("learningmodel: " + learningmodel)
    logging.info("x, y goal value: %s, %s", str(xgoal), str(ygoal))
    logging.info("functionname: " + functionname)
    logging.info("shared::printvalues Ended!")

def setLength(newValue):
    global length
    length = newValue

def setFunctionName(newValue):
    global functionname
    functionname = newValue

def setLearningModel(newValue):
    global learningmodel
    learningmodel = newValue

def setXGoal(newValue):
    global xgoal
    xgoal = newValue

def setYGoal(newValue):
    global ygoal
    ygoal = newValue
