import subprocess
import time

startFilePath = './start_ros.sh'
configFilePath = '/workspaces/MLSzakdoga/config/LEARN_DEFAULT_CONFIG.yaml'

subprocess.Popen(['bash', startFilePath])

time.sleep(15)

print("Sleep completed")

subprocess.run(['python3', 'src/main.py', configFilePath])
