from loguru import logger
from utils.trainGazebo import trainGazebo
from utils.continueTrainingGazebo import continueTrainingGazebo
from utils.saveDataFromTensorboardFiles import saveDataFromTensorboardFiles
from utils.createDirectories import createResourcesDirectories
from utils.sharedValues import sharedValues

def startFunction():
    sv = sharedValues()
    logger.info("startFunction started!")

    createResourcesDirectories()

    functionName = sv.functionName
    logger.info("functionName: " + str(functionName))
    
    if functionName == "Learn":
        trainGazebo()
    elif functionName == "Continue":
        continueTrainingGazebo()
    elif functionName == "SaveData":
        saveDataFromTensorboardFiles()
    else:
        raise ValueError(f"Unknown function: {functionName}")