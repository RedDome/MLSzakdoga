from loguru import logger

class sharedValues:
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

    @classmethod
    def printValues(cls):
        logger.info("shared::printValues Started!")
        logger.info("length: {} ", str(cls.length))
        logger.info("learningModel: {}", cls.learningModel)
        logger.info("x, y goal value: {}, {}", str(cls.xGoal), str(cls.yGoal))
        logger.info("functionName: {}", cls.functionName)
        logger.info("modelPath: {}", cls.modelPath)
        logger.info("shared::printValues Ended!")

    @classmethod
    def setLength(cls, newValue):
        cls.length = newValue

    @classmethod
    def setFunctionName(cls, newValue):
        cls.functionName = newValue

    @classmethod
    def setLearningModel(cls, newValue):
        cls.learningModel = newValue

    @classmethod
    def setXGoal(cls,newValue):
        cls.xGoal = newValue

    @classmethod
    def setYGoal(cls, newValue):
        cls.yGoal = newValue

    @classmethod
    def setModelPath(cls, newValue):
        cls.modelPath = newValue

    @classmethod
    def setCSVFilePath(cls, newValue):
        cls.csvFilePath = newValue

    @classmethod
    def setSaveDataAfterFinished(cls, newValue):
        cls.saveDataAfterFinished = newValue

    @classmethod
    def setLogFolder(cls, newValue):
        cls.logFolder = newValue

    @classmethod
    def setModelFolder(cls, newValue):
        cls.modelFolder = newValue  