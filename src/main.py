from gui.mainview import createMainGUI

import logging
import sys

from config.processconfigfile import processConfigFile

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s', handlers=[
    logging.FileHandler("app.log"),
    logging.StreamHandler()
])

if __name__ == "__main__":
    if len(sys.argv) == 2:
        path = sys.argv[1]
    else:
        raise ValueError(f"Incorrect amount of arguments! (1 is expected)")

    print(sys.argv)
    processConfigFile(path)