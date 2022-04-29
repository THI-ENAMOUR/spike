from typing import Dict
from uuid import UUID

from src.core.completion_checker import check_completion_before
from src.core.model.action.action import ActionList, Action
from src.core.model.action.atomic.generic.action_execution_list import ActionExecutionList
from src.core.model.action.atomic.generic.atomic_action import AtomicAction
from src.core.model.action.atomic.no_op_action import NoOpAction
from src.core.model.action.execution_method import ExecutionMethod
from src.core.model.action.group.action_group import ActionGroup
from src.util.action_duration import ActionDuration


class DefaultActionGroup(ActionGroup):
    def __init__(
        self,
        actions: ActionList = None,
        start_time_ms: int = 0,
        execution_method: ExecutionMethod = ExecutionMethod.SOLO,
    ):

        if actions is None:
            actions = []

        for action in actions:
            action.parent = self

        super().__init__(actions, start_time_ms, execution_method=execution_method)

        self.__selected_actions: ActionExecutionList = []
        self.__parents_of_selected_actions: Dict[UUID, ActionGroup] = {}

    def get_next_actions(self) -> (ActionExecutionList, Dict[UUID, ActionGroup]):
        # TODO: Write tests

        self.__selected_actions = []
        self.__parents_of_selected_actions = {}

        index = 0
        while self.actions and index < len(self.actions):
            action = self.actions[index]

            if action.completed:
                self.actions.pop(index)
                continue

            if check_completion_before(action):
                action.complete()
                self.actions.pop(index)
                continue

            if action.in_time_frame(self.time):
                continue_selection = self.select_action(action)
                if not continue_selection:
                    break
            else:
                break

            if action.completed:
                self.actions.pop(index)
                continue

            index = index + 1

        if len(self.__selected_actions) == 0:
            self.__selected_actions = [NoOpAction(start_time=self.time)]
            self.__parents_of_selected_actions = {self.id: self}

        if len(self.actions) == 0:
            self.completed = True
            self.__selected_actions = []
            self.__parents_of_selected_actions = {}

        # Return parent time, this imposes, that only actions from one parent can be returned
        return self.__selected_actions, self.__parents_of_selected_actions

    def update_time(self, delta_time: ActionDuration):
        for key, action in self.__parents_of_selected_actions.items():
            if self.id == key:
                self.time = self.time + delta_time
            else:
                action.update_time(delta_time)

    def select_action(self, action: Action) -> bool:

        if isinstance(action, ActionGroup):
            if action.execution_method == ExecutionMethod.MULTIPLE:
                self.__execute_action(action)
            elif action.execution_method == ExecutionMethod.NO_SAME_TYPE:
                raise ValueError("NO_SAME_TYPE cannot be chosen for action group")
            elif action.execution_method == ExecutionMethod.SOLO:
                if len(self.__selected_actions) == 0:
                    self.__execute_action(action)
                    if len(self.__selected_actions) == 0:
                        return True
                return False
        elif isinstance(action, AtomicAction):
            if action.execution_method == ExecutionMethod.MULTIPLE:
                self.__execute_action(action)
            elif action.execution_method == ExecutionMethod.NO_SAME_TYPE:
                contained_action_types = AtomicAction.map_to_action_types(self.__selected_actions)
                if action.action_type in contained_action_types:
                    raise ValueError("Primitive action with no_same_type execution conflicts with other running action")
                else:
                    self.__execute_action(action)
            elif action.execution_method == ExecutionMethod.SOLO:
                if len(self.__selected_actions) == 0:
                    self.__execute_action(action)
                    return False
                else:
                    raise ValueError("Primitive action with solo execution conflicts with other running action")
        else:
            raise ValueError

        return True

    def __execute_action(self, action: Action):
        if isinstance(action, ActionGroup):
            (child_actions, child_action_parents) = action.get_next_actions()
            self.__selected_actions.extend(child_actions)
            for id, actions in child_action_parents.items():
                self.__parents_of_selected_actions[id] = actions
        elif isinstance(action, AtomicAction):
            self.__selected_actions.append(action)
            self.__parents_of_selected_actions[self.id] = self
        else:
            raise ValueError
