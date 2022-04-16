from src.core.model.action.action import ActionList, Action
from src.core.model.action.atomic.generic.action_execution_list import ActionExecutionList
from src.core.model.action.atomic.generic.atomic_action import AtomicAction
from src.core.model.action.atomic.no_op_action import NoOpAction
from src.core.model.action.completion_checker import check_completion_before
from src.core.model.action.high_level.generic.action_group import ActionGroup
from src.util.action_duration import ActionDuration


class DefaultActionGroup(ActionGroup):
    def __init__(self, actions: ActionList = None):
        super().__init__()

        if actions is None:
            actions = []

        self.actions = actions
        self.time = ActionDuration(ns=0)

    def get_next_actions(self, delta_time: ActionDuration) -> (ActionExecutionList, ActionDuration):
        # TODO: Implement correctly and write tests
        # TODO: Move completed actions into a "completed list" to decrease necessary iterations

        self.time = self.time + delta_time

        selected_actions: ActionExecutionList = []

        index = 0
        while self.actions and index < len(self.actions):
            action = self.actions[index]

            if action.completed:
                self.actions.pop(index)
                continue

            if check_completion_before(action, self.time):
                action.complete()
                self.actions.pop(index)
                continue

            if action.is_selected(self.time):
                if not self.add_to_execution(selected_actions, action, delta_time):
                    break

            index = index + 1

        if len(selected_actions) == 0:
            selected_actions = [NoOpAction()]

        if len(self.actions) == 0:
            self.completed = True
            selected_actions = []

        # Return parent time, this imposes, that only actions from one parent can be returned
        return selected_actions, self.time

    @staticmethod
    def add_to_execution(selected_actions: ActionExecutionList, action: Action, delta_time: ActionDuration) -> bool:
        # TODO: Add execution type to algorithm

        if isinstance(action, ActionGroup):
            selected_actions.extend(action.get_next_actions(delta_time))
        elif isinstance(action, AtomicAction):
            selected_actions.append(action)
        else:
            raise ValueError

        # TODO: Just a quick solution for testing
        return not isinstance(action, ActionGroup)
