import yaml
import utils.commonvalues as cm
from utils.startfunction import startFunction
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
    with open(path, 'r') as f:
        data = yaml.full_load(f)

    functionName = data.get('FunctionName', DEFAULTS['FunctionName'])

    if functionName in ("None", ""):
        logger.error("FunctionName is not defined, check the config file! Program will be exiting now!")
        raise ValueError(f"FunctionName is not defined!")

    cm.setFunctionName(functionName)

    if functionName == "SaveData":
        processSaveData(data)
        cm.printvalues()
        startFunction()
        return

    if functionName == "Continue":
        processContinueData(data)

    cm.setXGoal(data.get('FunctionProperties', {}).get('XGoalPosition', DEFAULTS['XGoalPosition']))
    cm.setYGoal(data.get('FunctionProperties', {}).get('YGoalPosition', DEFAULTS['YGoalPosition']))
    cm.setLearningModel(data.get('FunctionProperties', {}).get('LearningModel', DEFAULTS['LearningModel']))
    cm.setLength(data.get('FunctionProperties', {}).get('Length', DEFAULTS['Length']))

    setSaveDataAfterFinishedValue = data.get('FunctionProperties', {}).get('SaveDataAfterFinished', DEFAULTS['SaveDataAfterFinished'])
    cm.setSaveDataAfterFinished(setSaveDataAfterFinishedValue)

    if setSaveDataAfterFinishedValue:
        cm.setCSVFilePath(data.get('FunctionProperties', {}).get('SaveDataProperties', {}).get('CsvFilePath', DEFAULTS['CsvFilePath']))

    cm.printvalues()

    startFunction()


def processSaveData(data):
    if data.get('FunctionProperties').get('LogFolder') == "":
        logger.error("LogFolder value not given, using default value!")

    cm.setLogFolder(data.get('FunctionProperties').get('LogFolder', DEFAULTS['LogFolder']))
    cm.setCSVFilePath(data.get('FunctionProperties').get('CsvFilePath', DEFAULTS['CsvFilePath']))

def processContinueData(data):
    cm.setModelPath(data.get('FunctionProperties').get('ModelPath', DEFAULTS['ModelPath']))
