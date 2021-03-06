from core.model.action.execution_method import ExecutionMethod
from error.illegal_state_error import IllegalStateError


class ExecutionListValidator(object):
    def validate(self, actions):
        """Validates an execution list. Throws error if validation error occurred."""
        action_types = set()
        for action in actions:
            # TODO: Think about if Pose and Navigation should have the same action type or not
            action_type = action.action_type

            if action.execution_method == ExecutionMethod.NO_SAME_TYPE and action_type in action_types:
                raise IllegalStateError("Action type already contained in action list for action " + str(action))
            elif action.execution_method == ExecutionMethod.EXCLUSIVE and len(actions) > 1:
                raise IllegalStateError("Action requires requires exclusive execution for action " + str(action))

            action_types.add(action_type)
