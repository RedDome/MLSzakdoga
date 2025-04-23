import subprocess
from loguru import logger
import os

logger.remove(0)
logger.add("app.log")

functionNameParam = os.environ.get("FUNCTION_NAME", "Missing")
logger.info("functionNameParam: {}", functionNameParam)
 
if functionNameParam == "Learn":
    configFilePath = '/workspaces/MLSzakdoga/config/LEARN_DEFAULT_CONFIG.yaml'
    logger.info("Runprogram with learn function started!")
    subprocess.run(['python3', 'src/main.py', configFilePath])  
elif functionNameParam == "Continue":
    configFilePath = '/workspaces/MLSzakdoga/config/CONTINUE_DEFAULT_CONFIG.yaml'
    logger.info("Runprogram with continue function started!")
    subprocess.run(['python3', 'src/main.py', configFilePath])      
elif functionNameParam == "SaveData":
    configFilePath = '/workspaces/MLSzakdoga/config/SAVE_DATA_DEFAULT_CONFIG.yaml'
    logger.info("Runprogram with savedata function started!")
    subprocess.run(['python3', 'src/main.py', configFilePath])
else:
    logger.info("FunctionName is missing or incorrect! Check your parameters! Program will be exiting now!")
    raise ValueError(f"FunctionName is missing or incorrect!")
