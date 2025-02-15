from loguru import logger

def exit_app(self):
    logger.info("Exiting program!")
    self.quit()