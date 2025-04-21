import yaml
from utils.sharedValues import sharedValues
from utils.startFunction import startFunction
from loguru import logger

DEFAULTS = {
    'FunctionName': 'None',
    'XGoalPosition': 2.0,
    'YGoalPosition': 2.0,
    'LearningModel': 'PPO',
    'Length' : 8000,
    'SaveDataAfterFinished' : False,
    'CsvFilePath': '/workspaces/MLSzakdoga/resources/processedData/tensorboard_data.csv'
}

def getDataWithDefault(data, key, default):
    value = data.get(key)
    if value is None or value == "":
        logger.info("Key: {} is empty, default value: {} will be used instead!", key, default)
        return default
    return value

def processConfigFile(path):
    sv = sharedValues()
    with open(path, 'r') as f:
        data = yaml.full_load(f)

    functionName = getDataWithDefault(data, 'FunctionName', DEFAULTS['FunctionName'])

    if functionName in ("None", ""):
        logger.error("FunctionName is not defined, check the config file! Program will be exiting now!")
        raise ValueError(f"FunctionName is not defined!")

    sv.setFunctionName(functionName)

    functionPropertiesData = data.get('FunctionProperties')

    if functionName == "SaveData":
        processSaveData(functionPropertiesData)
        sv.printValues()
        startFunction()
        return

    if functionName == "Continue":
        processContinueData(functionPropertiesData)

    functionPropertiesData = data.get('FunctionProperties')
    sv.setXGoal(getDataWithDefault(functionPropertiesData, 'XGoalPosition', DEFAULTS['XGoalPosition']))
    sv.setYGoal(getDataWithDefault(functionPropertiesData, 'YGoalPosition', DEFAULTS['YGoalPosition']))
    sv.setLearningModel(getDataWithDefault(functionPropertiesData, 'LearningModel', DEFAULTS['LearningModel']))
    sv.setLength(getDataWithDefault(functionPropertiesData, 'Length', DEFAULTS['Length']))

    setSaveDataAfterFinishedValue = getDataWithDefault(functionPropertiesData, 'SaveDataAfterFinished', DEFAULTS['SaveDataAfterFinished'])
    sv.setSaveDataAfterFinished(setSaveDataAfterFinishedValue)

    if setSaveDataAfterFinishedValue:
        saveDataPropertiesData = functionPropertiesData.get('SaveDataProperties')
        sv.setCSVFilePath(getDataWithDefault(saveDataPropertiesData, 'CsvFilePath', DEFAULTS['CsvFilePath']))

    sv.printValues()

    startFunction()


def processSaveData(functionPropertiesData):
    sv = sharedValues()
    if not functionPropertiesData.get('LogFolder'):
        logger.error("LogFolder is not defined, check the config file! Program will be exiting now!")
        raise ValueError(f"LogFolder is not defined!")

    sv.setLogFolder(functionPropertiesData.get('LogFolder'))
    sv.setCSVFilePath(getDataWithDefault(functionPropertiesData, 'CsvFilePath', DEFAULTS['CsvFilePath']))

def processContinueData(functionPropertiesData):
    sv = sharedValues()
    if not functionPropertiesData.get('ModelPath'):
        logger.error("ModelPath is not defined, check the config file! Program will be exiting now!")
        raise ValueError(f"ModelPath is not defined!")

    sv.setModelPath(functionPropertiesData.get('ModelPath'))
