import utils.commonvalues as cm
from datetime import datetime
import os
from loguru import logger

def createDirectories():
    modelsDirectory = f"resources/models/{cm.learningModel}"
    logDirectory = f"resources/logs/{cm.learningModel}"

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
            cm.setModelFolder(modelFolder)
            cm.setLogFolder(logFolder)
            break
        else:
            suffix += 1