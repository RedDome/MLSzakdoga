import subprocess
from loguru import logger

logger.remove(0)
logger.add("app.log")

startFilePath = './start_ros.sh'
configFilePath = '/workspaces/MLSzakdoga/config/LEARN_DEFAULT_CONFIG.yaml'

logger.info("runprogram started!")

subprocess.run(['python3', 'src/main.py', configFilePath])
