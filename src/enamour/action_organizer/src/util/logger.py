import logging


class Logger:
    """Provides functions to print logs to the console and log files (log files are located in spike/logs"""

    def __init__(self, name):
        self.name = name
        self.logger = logging.getLogger(name)
        self.c_handler = logging.StreamHandler()
        self.f_handler = logging.FileHandler("../../../logs/" + self.name + ".log")

        self.logger.setLevel(logging.DEBUG)
        self.c_handler.setLevel(logging.DEBUG)
        self.f_handler.setLevel(logging.ERROR)
        self.logger.addHandler(self.c_handler)
        self.logger.addHandler(self.f_handler)

    def info(self, message):
        self.logger.info(f"INFO - [{self.name}]: \n{message}")

    def warning(self, message):
        self.logger.warning(f"WARNING - [{self.name}]: \n{message}")

    def error(self, message):
        self.logger.error(f"ERROR - [{self.name}]: \n{message}")

    def debug(self, message):
        self.logger.debug(f"DEBUG - [{self.name}]: \n{message}")

    def critical(self, message):
        self.logger.critical(f"CRITICAL - [{self.name}]: \n{message}")
