from error.application_error import ApplicationError
from error.handler.error_handler import ErrorHandler
from error.illegal_state_error import IllegalStateError
from util.logger import Logger


class ActionOrganizerErrorHandler(ErrorHandler):
    """Handle errors occurring in the action organizer."""

    __logger = Logger(__name__)

    def __init__(self, action_queue):
        self.action_queue = action_queue

    def handle(self, error, **kwargs):
        try:
            action = kwargs["action"]
        except ValueError:
            raise IllegalStateError(
                "No action provided for action organizer error handler. Cause of invocation: " + str(error)
            )

        self.__logger.error("Error occurred in action organizer with message: " + str(error))

        if isinstance(error, ApplicationError):
            if error.is_critical:
                self.__delete_action_from_queue(action)
        else:
            self.__delete_action_from_queue(action)

    def __delete_action_from_queue(self, action):
        if action is not None:
            self.action_queue.pop_on_error(action)
