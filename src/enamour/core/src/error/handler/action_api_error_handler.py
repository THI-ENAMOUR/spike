from error.handler.error_handler import ErrorHandler
from util.logger import Logger


class ActionApiErrorHandler(ErrorHandler):
    """Handle errors occurring in the action api service."""

    __logger = Logger(__name__)

    def handle(self, error: BaseException, **kwargs):
        self.__logger.error(f"Error occurred in action api service with message: {error}")
