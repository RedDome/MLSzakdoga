import subprocess

startFilePath = './start_ros.sh'
configFilePath = '/workspaces/MLSzakdoga/config/LEARN_DEFAULT_CONFIG.yaml'

print("Sleep completed")

subprocess.run(['python3', 'src/main.py', configFilePath])
