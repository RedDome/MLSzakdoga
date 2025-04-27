from utils.sharedValues import sharedValues
from datetime import datetime
import os
from loguru import logger
import csv

def createResourcesDirectories():
    sv = sharedValues()
    modelsDirectory = "resources/models"
    logDirectory = "resources/logs"
    processedDataDirectory = "resources/processedData"

    for directory in [modelsDirectory, logDirectory, processedDataDirectory]:
        if not os.path.exists(directory):
            os.makedirs(directory)

    if not os.path.exists(sv.csvFilePath):
        with open(sv.csvFilePath, mode='w', newline='') as csvfile:
            writer = csv.writer(csvfile)

    logger.info("Resources folders and csv file created!")    

def createTrainingDirectories():
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