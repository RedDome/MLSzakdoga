from utils.commonvalues import learningModel, setLogFolder, setModelFolder
from datetime import datetime
import os
import logging

def createDirectories():
    modelsDirectory = f"resources/models/{learningModel}"
    logDirectory = f"resources/logs/{learningModel}"

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
            logging.info("Created Model Folder: " + modelFolder)
            logging.info("Created Log Folder: " + logFolder)
            setModelFolder(modelFolder)
            setLogFolder(logFolder)
            break
        else:
            suffix += 1