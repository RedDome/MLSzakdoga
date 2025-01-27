import yaml

def processConfigFile():
    with open('/workspaces/MLSzakdoga/config/LEARN_DEFAULT_CONFIG.yaml', 'r') as f:
        data = yaml.full_load(f)
    
    # Print the values as a dictionary
    output = {
        'FunctionName': data.get('FunctionName'),
        'XGoalPosition': data.get('XGoalPosition'),
        'YGoalPosition': data.get('YGoalPosition'),
        'LearningModel': data.get('LearningModel', 'Test')
    }

    print(output)