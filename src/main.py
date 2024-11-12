from gui.mainview import create_main_view

import logging

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s', handlers=[
    logging.FileHandler("app.log"),
    logging.StreamHandler()
])

if __name__ == "__main__":
    create_main_view()