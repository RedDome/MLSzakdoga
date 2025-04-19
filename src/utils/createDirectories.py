from utils.sharedValues import sharedValues
from datetime import datetime
import os
from loguru import logger

def createDirectories():
    sv = sharedValues()
    modelsDirectory = f"resources/models/{sv.learningModel}"
    logDirectory = f"resources/logs/{sv.learningModel}"

    if not os.path.exists(modelsDirectory):
        os.makedirs(modelsDirectory)

    if not os.path.exists(logDirectory):
        os.makedirs(logDirectory)

    todayDate = datetime.now().strftime("%Y-%m-%d")
    modelFolder = ""
    logFolder = ""

    suffix = 1
    while True:
        folderName = f"{todayDate}_{str(suffix).zfill(2)}"
        modelFolder = os.path.join(modelsDirectory, folderName)
        logFolder = os.path.join(logDirectory, folderName)

        if not os.path.exists(modelFolder) and not os.path.exists(logFolder) :
            os.makedirs(modelFolder)
            os.makedirs(logFolder)
            logger.info("Created Model Folder: " + modelFolder)
            logger.info("Created Log Folder: " + logFolder)
            sv.setModelFolder(modelFolder)
            sv.setLogFolder(logFolder)
            break
        else:
            suffix += 1