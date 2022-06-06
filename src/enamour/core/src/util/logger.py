import logging
import os
import sys


def setup_log_dir(log_location):
    try:
        os.makedirs(log_location)
    except OSError:
        pass


class InfoLogLevelFilter(logging.Filter):
    def filter(self, record):
        return record.levelno <= logging.INFO


class Logger(object):
    """Provides functions to print logs to the console and log files"""

    formatter = logging.Formatter(fmt="%(levelname)s [%(name)s]: %(message)s")

    # File logger configuration
    file_name = "core.log"
    file_handler = logging.FileHandler(file_name, mode="w", delay=True)
    file_handler.setLevel(logging.DEBUG)
    file_handler.formatter = formatter

    @staticmethod
    def setup_file_logger(logging_dir=None):
        file_location = os.path.join(logging_dir, Logger.file_name) if logging_dir is not None else Logger.file_name
        if logging_dir is not None:
            setup_log_dir(logging_dir)
        Logger.file_handler = logging.FileHandler(file_location, mode="w", delay=True)
        Logger.file_handler.setLevel(logging.DEBUG)
        Logger.file_handler.formatter = Logger.formatter

    def __init__(self, name, log_level=logging.INFO):
        self.name = name
        self.logger = logging.getLogger(name)
        self.logger.setLevel(log_level)

        info_console_handler = logging.StreamHandler(sys.stdout)
        info_console_handler.setLevel(logging.DEBUG)
        info_console_handler.formatter = Logger.formatter
        info_console_handler.addFilter(InfoLogLevelFilter())
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
