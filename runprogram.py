import subprocess
import time

sh_file_path = './start_ros.sh'
config_file_path = '/workspaces/MLSzakdoga/config/LEARN_DEFAULT_CONFIG.yaml'

subprocess.Popen(['bash', sh_file_path])

time.sleep(15)

print("Sleep completed")

subprocess.run(['python3', 'src/main.py', config_file_path])
