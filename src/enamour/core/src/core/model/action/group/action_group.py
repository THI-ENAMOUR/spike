import abc
from typing import List

from core.completion_checker import check_completion_before_selection
from core.model.action.action import Action, ActionList
from core.model.action.atomic.atomic_action import AtomicAction
from core.model.action.atomic.no_op_action import NoOpAction
from core.model.action.execution_method import ExecutionMethod
from core.model.action.timing_option import TimingOption, StartTime
from core.model.common.time_stamp import TimeStamp
from error.illegal_state_error import IllegalStateError


class ActionGroup(Action, metaclass=abc.ABCMeta):
    """Used to recursively wrap actions within each other by containing a list of actions.
    The actions are sorted by starting time."""

    def __init__(
        self,
        actions: ActionList = None,
        start_ms: int = 0,
        timing_option: TimingOption = None,
        execution_method: ExecutionMethod = ExecutionMethod.EXCLUSIVE,
    ):
        if timing_option is None:
            timing_option = StartTime(start_ms=start_ms)

        super().__init__(timing_option, execution_method=execution_method)

        actions = actions if actions is not None else []
        for action in actions:
            action.parent = self
        self.actions = actions
        # IMPORTANT: Sort the action list, so we can make assumptions based on the sorting of the start time
        self.sort()

        self.time = TimeStamp(ns=0)
        self.__execution_list: List[AtomicAction] = []

    def get_next_actions(self) -> List[AtomicAction]:
        """Returns the next to-be-executed actions. If the list is empty, this means the action group is finished"""

        self.__execution_list = []
        index = 0
        while self.actions and index < len(self.actions):
            action = self.actions[index]

            # Remove completed actions from action group to increase performance
            if action.completed:
                self.actions.pop(index)
                continue

            # Check if the action should be completed at the current time stamp
            if check_completion_before_selection(action):
                action.complete()
                self.actions.pop(index)
                continue

            # Check if the action should be selected with the current time
            if action.in_time_frame(self.time):
                continue_selection = self.select_action(action)
                if not continue_selection:
                    break
            else:
                break

            # Need to check completion again, since contained action groups could now be completed but weren't so before
            if action.completed:
                self.actions.pop(index)
                continue

            index = index + 1

        if self.is_execution_list_empty():
            # Add NoOpAction to execution list since no action is being executed at the current time stamp
            self.__execution_list = [NoOpAction(start_time_ms=self.time.to_ms(), parent=self)]

        if len(self.actions) == 0:
            # If no actions are in the action list anymore, this means the execution of this action group is compled
            self.completed = True
            self.__execution_list = []

        return self.__execution_list

    def select_action(self, action: Action) -> bool:
        if isinstance(action, ActionGroup):
            return self.__select_action_group(action)
        elif isinstance(action, AtomicAction):
            return self.__select_atomic_action(action)
        else:
            raise IllegalStateError(f"Action {action.id} is an unknown action instance")

    def __select_action_group(self, action: "ActionGroup") -> bool:
        if action.execution_method == ExecutionMethod.MULTIPLE:
            self.__execute_action_group(action)
        elif action.execution_method == ExecutionMethod.NO_SAME_TYPE:
            raise IllegalStateError("NO_SAME_TYPE is not supported for action group")
        elif action.execution_method == ExecutionMethod.EXCLUSIVE:
            if self.is_execution_list_empty():
                self.__execute_action_group(action)
                if not self.is_execution_list_empty():
                    # Stop the selection only if the action group actually added actions to the execution list
                    return False
            else:
                # Can only be executed solo but there are already actions in the execution list
                return False
        return True

    def __select_atomic_action(self, action: AtomicAction) -> bool:
        if action.execution_method == ExecutionMethod.MULTIPLE:
            self.__execute_atomic_action(action)
        elif action.execution_method == ExecutionMethod.NO_SAME_TYPE:
            contained_action_types = AtomicAction.to_action_type_set(self.__execution_list)
            if action.action_type in contained_action_types:
                # Raise error since the execution of atomic actions cannot be delayed
                raise IllegalStateError(
                    f"Atomic action {action.id} with no_same_type execution conflicts with other actions"
                )
            else:
                self.__execute_atomic_action(action)
        elif action.execution_method == ExecutionMethod.EXCLUSIVE:
            if self.is_execution_list_empty():
                self.__execute_atomic_action(action)
                return False
            else:
                # Raise error since the execution of atomic actions cannot be delayed
                raise IllegalStateError(f"Atomic action {action.id} with solo execution conflicts with other actions")
        return True

    def update_parent_time_of_executed_actions(self, delta_time: TimeStamp):
        parent_set = set()
        for action in self.__execution_list:
            parent = action.parent
            if parent is None:
                raise IllegalStateError(f"Action {action.id} has no parent")
            elif parent.id == self.id:
                self.time = self.time + delta_time
            elif parent not in parent_set:
                parent.update_parent_time_of_executed_actions(delta_time)
                parent_set.add(parent)

    def is_execution_list_empty(self):
        return len(self.__execution_list) == 0

    def __execute_action_group(self, action: "ActionGroup"):
        self.__execution_list.extend(action.get_next_actions())

    def __execute_atomic_action(self, action: AtomicAction):
        self.__execution_list.append(action)

    def sort(self):
        TimingOption.sort(self.actions)
