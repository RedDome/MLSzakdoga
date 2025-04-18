from loguru import logger
from utils.train import train
from utils.continuetraining import continueTrainingGazebo
from utils.saveDataFromTensorboardFiles import saveDataFromTensorboardFiles
import utils.commonvalues

def startFunction():
    logger.info("startFunction started!")
    functionName = utils.commonvalues.functionName 
    logger.info("functionName: " + str(functionName))
    
    if functionName == "Learn":
        train()
    elif functionName == "Continue":
        continueTrainingGazebo()
    elif functionName == "SaveData":
        saveDataFromTensorboardFiles()
    else:
        raise ValueError(f"Unknown function: {functionName}")