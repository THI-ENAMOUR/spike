from typing import Optional, Callable, TypeVar, List

from core.model.action.group.action_group import ActionGroup
from core.validation.action_group_validator import ActionGroupValidator
from util.synchronized import synchronized

T = TypeVar("T")


# TODO: Write test cases including thread-safety
@synchronized
class ActionQueue:
    """Provides a thread-safe FIFO queue for ActionGroups."""

    def __init__(
        self,
        latest_completed_actions_list_size: int = 5,
        action_validator: ActionGroupValidator = ActionGroupValidator(),
    ):
        if latest_completed_actions_list_size < 0:
            latest_completed_actions_list_size = 0
        self.latest_completed_actions_list_size = latest_completed_actions_list_size

        self.action_validator = action_validator
        self.queue: List[ActionGroup] = []
        self.latest_completed_actions: List[ActionGroup] = []
        """ List of latest completed actions, lower means more recently completed.
            The max size is defined by recent_completed_actions_list_size."""

    def peek_and_pop_completed(self) -> Optional[ActionGroup]:
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

    def push(self, *actions: ActionGroup):
        """Validates actions and pushes them to the end of the queue."""
        for action in actions:
            self.action_validator.validate(action)
            self.queue.append(action)

    def lock_queue_for_execution(self, func: Callable[["ActionQueue"], T]) -> T:
        """Locks the queue for all threads while running the provided function."""
        return func(self)

    def pop(self, filter_fun: Callable[[int, ActionGroup], bool]):
        """Removes actions which statisfy the provided filter function from the queue and
        adds them to recent_completed_actions."""
        removed_items = [self.queue.pop(index) for index, item in enumerate(self.queue) if filter_fun(index, item)]
        self.__add_to_latest_completed_list(*removed_items)

    def __pop_by_index(self, index):
        action = self.queue.pop(index)
        self.__add_to_latest_completed_list(action)

    def __add_to_latest_completed_list(self, *actions: ActionGroup):
        if self.latest_completed_actions_list_size > 0:
            for action in actions:
                print("Completed action list with id " + str(action.id))
                self.latest_completed_actions.insert(0, action)
            while len(self.latest_completed_actions) > self.latest_completed_actions_list_size:
                self.latest_completed_actions.pop()
