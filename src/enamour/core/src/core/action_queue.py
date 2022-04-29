import typing
from typing import Optional, Callable

from src.core.model.action.group.action_group import ActionGroup
from src.util.synchronized import synchronized

T = typing.TypeVar("T")


# TODO: Write test cases including thread-safety
@synchronized
class ActionQueue:
    """Provides a thread-safe FIFO queue for ActionGroups."""

    def __init__(self, recent_completed_actions_list_size: int = 5):
        if recent_completed_actions_list_size < 0:
            recent_completed_actions_list_size = 0

        self.queue: "list[ActionGroup]" = []
        self.recent_completed_actions_list_size = recent_completed_actions_list_size
        self.recent_completed_actions: "list[ActionGroup]" = []
        """ List of recently completed actions, sorted from most recent to oldest.
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
        """Pushes new actions to the end of the queue."""
        for action in actions:
            self.queue.append(action)

    def run_exclusive(self, func: Callable[["ActionQueue"], T]) -> T:
        """Locks and synchronizes the queue across all threads while running the provided function."""
        return func(self)

    def pop(self, filter_fun: Callable[[int, ActionGroup], bool]):
        """Removes actions which statisfy the provided filter function from the queue and
        adds them to recent_completed_actions."""
        removed_items = [self.queue.pop(index) for index, item in enumerate(self.queue) if filter_fun(index, item)]
        self.__add_to_completed_actions(*removed_items)

    def __pop_by_index(self, index):
        action = self.queue.pop(index)
        self.__add_to_completed_actions(action)

    def __add_to_completed_actions(self, *actions: ActionGroup):
        if self.recent_completed_actions_list_size > 0:
            for action in actions:
                print("Completed action list with id " + str(action.id))
                self.recent_completed_actions.insert(0, action)
            while len(self.recent_completed_actions) > self.recent_completed_actions_list_size:
                self.recent_completed_actions.pop()
