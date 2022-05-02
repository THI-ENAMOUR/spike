import logging
import os
import string
import sys


class InfoFilter:
    pass


class Logger:
    """Provides functions to print logs to the console and log files (log files are located in spike/logs"""

    log_dir = "../logs/"
    file_handler = None
    formatter = logging.Formatter(fmt="%(levelname)s [%(name)s]: %(message)s")

    def __init__(self, name, log_level=logging.INFO):
        self.name = name
        self.logger = logging.getLogger(name)
        self.logger.setLevel(log_level)

        if Logger.file_handler is None:
            Logger.update_file_logger(Logger.log_dir)

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
        self.logger.error(message)

    def debug(self, message):
        self.logger.debug(message)

    def critical(self, message):
        self.logger.critical(message)

    @staticmethod
    def update_file_logger(directory: string, file_name="core.log", file_handler_log_level=logging.DEBUG):
        Logger.log_dir = directory
        try:
            os.makedirs(Logger.log_dir)
        except FileExistsError:
            pass
        file_handler = logging.FileHandler(f"{Logger.log_dir}/{file_name}", mode="w")
        file_handler.setLevel(file_handler_log_level)
        file_handler.formatter = Logger.formatter
        Logger.file_handler = file_handler
