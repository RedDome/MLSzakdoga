import yaml
from utils.sharedValues import sharedValues
from utils.startFunction import startFunction
from loguru import logger

DEFAULTS = {
    'FunctionName': 'None',
    'XGoalPosition': 2.0,
    'YGoalPosition': 2.0,
    'LearningModel': 'Test',
    'Length' : 10000,
    'SaveDataAfterFinished' : False,
    'ModelPath': '/workspaces/MLSzakdoga/resources/models/PPO/10000.zip',
    'CsvFilePath': '/workspaces/MLSzakdoga/resources/processedData/tensorboard_data.csv',
    'LogFolder': '/workspaces/MLSzakdoga/resources/logs/PPO_0'
}

def processConfigFile(path):
    sv = sharedValues()
    with open(path, 'r') as f:
        data = yaml.full_load(f)

    functionName = data.get('FunctionName', DEFAULTS['FunctionName'])

    if functionName in ("None", ""):
        logger.error("FunctionName is not defined, check the config file! Program will be exiting now!")
        raise ValueError(f"FunctionName is not defined!")

    sv.setFunctionName(functionName)

    if functionName == "SaveData":
        processSaveData(data)
        sv.printValues()
        startFunction()
        return

    if functionName == "Continue":
        processContinueData(data)

    sv.setXGoal(data.get('FunctionProperties', {}).get('XGoalPosition', DEFAULTS['XGoalPosition']))
    sv.setYGoal(data.get('FunctionProperties', {}).get('YGoalPosition', DEFAULTS['YGoalPosition']))
    sv.setLearningModel(data.get('FunctionProperties', {}).get('LearningModel', DEFAULTS['LearningModel']))
    sv.setLength(data.get('FunctionProperties', {}).get('Length', DEFAULTS['Length']))

    setSaveDataAfterFinishedValue = data.get('FunctionProperties', {}).get('SaveDataAfterFinished', DEFAULTS['SaveDataAfterFinished'])
    sv.setSaveDataAfterFinished(setSaveDataAfterFinishedValue)

    if setSaveDataAfterFinishedValue:
        sv.setCSVFilePath(data.get('FunctionProperties', {}).get('SaveDataProperties', {}).get('CsvFilePath', DEFAULTS['CsvFilePath']))

    sv.printValues()

    startFunction()


def processSaveData(data):
    sv = sharedValues()
    if data.get('FunctionProperties').get('LogFolder') == "":
        logger.error("LogFolder value not given, using default value!")

    sv.setLogFolder(data.get('FunctionProperties').get('LogFolder', DEFAULTS['LogFolder']))
    sv.setCSVFilePath(data.get('FunctionProperties').get('CsvFilePath', DEFAULTS['CsvFilePath']))

def processContinueData(data):
    sv = sharedValues()
    sv.setModelPath(data.get('FunctionProperties').get('ModelPath', DEFAULTS['ModelPath']))
