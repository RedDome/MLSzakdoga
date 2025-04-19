from loguru import logger

logger.remove(0)
logger.add("app.log")

import sys

from config.processConfigFile import processConfigFile

if __name__ == "__main__":
    logger.info("Main started")
    if len(sys.argv) == 2:
        path = sys.argv[1]
    else:
        raise ValueError(f"Incorrect amount of arguments! (1 is expected)")

    logger.info(sys.argv)
    processConfigFile(path)