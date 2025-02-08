import yaml
from utils.commonvalues import setFunctionName, setLearningModel, setLength, setXGoal, setYGoal, setModelPath, printvalues, setTensorboardDataPath
from utils.startfunction import startFunction

def processConfigFile(path):
    with open(path, 'r') as f:
        data = yaml.full_load(f)
    
    # Print the values as a dictionary
    output = {
        'FunctionName': data.get('FunctionName', 'None'),
        'XGoalPosition': data.get('XGoalPosition', 1),
        'YGoalPosition': data.get('YGoalPosition', 1),
        'LearningModel': data.get('LearningModel', 'Test'),
        'ModelPath': data.get('ModelPath', ''),
        'TensorboardDataPath' : data.get('TensorboardDataPath', '')
    }

    print(output)

    printvalues()

    setFunctionName(output["FunctionName"])
    setXGoal(output["XGoalPosition"])
    setYGoal(output["YGoalPosition"])
    setLearningModel(output["LearningModel"])
    setModelPath(output["ModelPath"])
    setTensorboardDataPath(output["TensorboardDataPath"])

    printvalues()

    startFunction()


