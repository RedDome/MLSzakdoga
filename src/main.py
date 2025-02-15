from loguru import logger

logger.remove(0)
logger.add("app.log")

import sys

from config.processconfigfile import processConfigFile

if __name__ == "__main__":
    if len(sys.argv) == 2:
        path = sys.argv[1]
    else:
        raise ValueError(f"Incorrect amount of arguments! (1 is expected)")

    logger.info(sys.argv)
    processConfigFile(path)