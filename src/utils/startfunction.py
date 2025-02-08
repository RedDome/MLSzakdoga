import logging
from utils.train import train
from utils.continuetraining import continueTrainingGazebo
from utils.saveDataFromTensorboardFiles import saveDataFromTensorboardFiles
import utils.commonvalues

def startFunction():
    logging.info("startFunction started!")
    functionName = utils.commonvalues.functionName 
    logging.info("functionName: " + str(functionName))
    
    if functionName == "Learn":
        train()
    elif functionName == "Continue":
        continueTrainingGazebo()
    elif functionName == "Capture":
        logging.info("TBD")
        # captureGazebo()
    elif functionName == "SaveData":
        saveDataFromTensorboardFiles()
    else:
        raise ValueError(f"Unknown function: {functionName}")