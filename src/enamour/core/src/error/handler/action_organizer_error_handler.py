from typing import Optional

from core.action_queue import ActionQueue
from core.model.action.group.action_group import ActionGroup
from error.application_error import ApplicationError
from error.handler.error_handler import ErrorHandler
from error.illegal_state_error import IllegalStateError
from util.logger import Logger


class ActionOrganizerErrorHandler(ErrorHandler):
    __logger = Logger(__name__)

    def __init__(self, action_queue: ActionQueue):
        self.action_queue = action_queue

    def handle(self, error: BaseException, **kwargs):
        try:
            action = kwargs["action"]
        except ValueError:
            raise IllegalStateError(
                f"No action provided for action organizer error handler. Cause of invocation: {error}"
            )

        self.__logger.warning(f"Error occurred in action organizer with message: {error}")

        if isinstance(error, ApplicationError):
            if error.is_critical:
                self.__delete_action_from_queue(action)
        else:
            self.__delete_action_from_queue(action)

    def __delete_action_from_queue(self, action: Optional[ActionGroup]):
        if action is not None:
            self.action_queue.pop_on_error(action)
