from core.validation.action_group_validator import ActionGroupValidator
from util.logger import Logger
from util.synchronized import synchronized


@synchronized
class ActionQueue(object):
    """Provides a thread-safe FIFO queue for ActionGroups."""

    __logger = Logger(__name__)

    def __init__(
        self,
        latest_completed_actions_list_size=5,
        latest_error_actions_list_size=5,
        action_validator=ActionGroupValidator(),
    ):
        if latest_completed_actions_list_size < 0:
            latest_completed_actions_list_size = 0
        self.latest_completed_actions_list_size = latest_completed_actions_list_size

        if latest_error_actions_list_size < 0:
            latest_error_actions_list_size = 0
        self.latest_error_actions_list_size = latest_error_actions_list_size

        self.action_validator = action_validator
        self.queue = []
        self.latest_completed_actions = []
        """ List of latest completed actions, lower means more recently completed.
                    The max size is defined by recent_completed_actions_list_size."""
        self.latest_error_actions = []
        """ List of latest actions that got deleted based on an error, lower means more recently completed.
                    The max size is defined by latest_error_actions_list_size."""

    def peek_and_pop_completed(self):
        """Until the first uncompleted action is encountered pops all completed actions, then returns it.
        Don't make any assumptions based on the returned action, because it could already be modified bit another
        thread."""

        while len(self.queue) > 0:
            current_action = self.queue[0]
            if current_action.completed:
                self.__pop_by_index(0)
            else:
                return current_action

        return None

    def push(self, *actions):
        """Validates actions and pushes them to the end of the queue."""
        for action in actions:
            self.action_validator.validate(action)
            self.__logger.info("Add action {id} to action queue".format(id=action.id))
            self.queue.append(action)

    def lock_queue_for_execution(self, func):
        """Locks the queue for all threads while running the provided function."""
        return func(self)

    def pop_on_error(self, *actions):
        for action in actions:
            for index, item in enumerate(self.queue):
                if item.id == action.id:
                    self.__logger.warning("Delete action {id} from queue due to error".format(id=action.id))
                    self.queue.pop(index)
                    self.__add_to_latest_error_list(action)

    def __pop_by_index(self, index):
        action = self.queue.pop(index)
        self.__add_to_latest_completed_list(action)

    def pop_all_actions(self):
        while len(self.queue) > 0:
            self.__pop_by_index(0)

    def __add_to_latest_completed_list(self, *actions):
        if self.latest_completed_actions_list_size > 0:
            for action in actions:
                self.__logger.info("Completed action list with id " + str(action.id))
                self.latest_completed_actions.insert(0, action)
            while len(self.latest_completed_actions) > self.latest_completed_actions_list_size:
                self.latest_completed_actions.pop()

    def __add_to_latest_error_list(self, *actions):
        if self.latest_error_actions_list_size > 0:
            for action in actions:
                self.__logger.info("Add action to error list with id " + str(action.id))
                self.latest_error_actions.insert(0, action)
            while len(self.latest_error_actions) > self.latest_error_actions_list_size:
                self.latest_error_actions.pop()
