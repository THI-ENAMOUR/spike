from error.handler.error_handler import ErrorHandler
from error.illegal_argument_error import IllegalArgumentError
from error.validation_error import ValidationError
from util.logger import Logger


class ActionApiErrorHandler(ErrorHandler):
    """Handle errors occurring in the action api service."""

    __logger = Logger(__name__)

    def handle(self, error, **kwargs):
        if isinstance(error, IllegalArgumentError):
            error = ValidationError(message=error.message, reason=None)

        self.__logger.error("Error occurred in action api service with message: " + str(error))
