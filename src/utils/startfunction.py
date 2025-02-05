import logging
from utils.train import train
from utils.continuetraining import continueTrainingGazebo
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
    else:
        raise ValueError(f"Unknown function: {functionName}")