from core.model.action.atomic.generic.action_execution_list import ActionExecutionList
from core.model.action.execution_method import ExecutionMethod
from error.illegal_state_error import IllegalStateError


class ExecutionListValidator:
    def validate(self, actions: ActionExecutionList):
        """Validates an execution list. Throws error if validation error occurred."""
        action_types = set()
        for action in actions:
            # TODO: Think about if Pose and Navigation should have the same action type or not
            action_type = action.action_type if not action.action_type.is_movement_action() else "movement"

            if action.execution_method == ExecutionMethod.NO_SAME_TYPE and action_type in action_types:
                raise IllegalStateError(f"Action type already contained in action list for action {action}")
            elif action.execution_method == ExecutionMethod.SOLO and len(actions) > 1:
                raise IllegalStateError(f"Action requires requires exclusive execution for action {action}")

            action_types.add(action_type)
