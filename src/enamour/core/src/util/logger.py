import logging
import os
import sys

from util.config import Config


def setup_log_dir(log_location):
    try:
        os.makedirs(log_location)
    except FileExistsError:
        pass


class Logger:
    """Provides functions to print logs to the console and log files"""

    formatter = logging.Formatter(fmt="%(levelname)s [%(name)s]: %(message)s")

    # File logger configuration
    log_location = Config.log_location
    setup_log_dir(log_location)
    file_name = "core.log"
    file_handler = logging.FileHandler(os.path.join(log_location, file_name), mode="w")
    file_handler.setLevel(logging.DEBUG)
    file_handler.formatter = formatter

    def __init__(self, name, log_level=logging.INFO):
        self.name = name
        self.logger = logging.getLogger(name)
        self.logger.setLevel(log_level)

        info_console_handler = logging.StreamHandler(sys.stdout)
        info_console_handler.setLevel(logging.DEBUG)
        info_console_handler.formatter = Logger.formatter
        info_console_handler.addFilter(lambda record: record.levelno <= logging.INFO)
        error_console_handler = logging.StreamHandler()
        error_console_handler.setLevel(logging.WARNING)
        error_console_handler.formatter = Logger.formatter

        self.logger.addHandler(info_console_handler)
        self.logger.addHandler(error_console_handler)
        self.logger.addHandler(Logger.file_handler)

    def info(self, message):
        self.logger.info(message)

    def warning(self, message):
        self.logger.warning(message)

    def error(self, message):
        self.logger.error(message, exc_info=True)

    def debug(self, message):
        self.logger.debug(message)

    def critical(self, message):
        self.logger.critical(message)
