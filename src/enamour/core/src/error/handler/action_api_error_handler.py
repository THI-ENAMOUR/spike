from error.handler.error_handler import ErrorHandler
from util.logger import Logger


class ActionApiErrorHandler(ErrorHandler):
    """Handle errors occurring in the action api service."""

    __logger = Logger(__name__)

    def handle(self, error, **kwargs):
        self.__logger.error("Error occurred in action api service with message: " + str(error))
