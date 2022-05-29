from core.model.action.group.action_group import ActionGroup
from error.illegal_argument_error import IllegalArgumentError
from error.validation_error import ValidationError
from util.logger import Logger


class ActionGroupValidator(object):
    __logger = Logger(__name__)

    def validate(self, action_group):
        """Validates an action group. Throws error if validation error occurred."""
        self.__logger.info("Validate action group " + str(action_group.id))

        if not isinstance(action_group, ActionGroup):
            raise IllegalArgumentError("Provided action group for validation is not of type 'ActionGroup'")

        ActionGroupValidator.validate_action_group_start_time_sorting(action_group.actions)

    @staticmethod
    def validate_action_group_start_time_sorting(actions):
        def to_start_time(action):
            if isinstance(action, ActionGroup):
                ActionGroupValidator.validate_action_group_start_time_sorting(action.actions)
            return action.timing_option.get_start_time()

        start_times = list(map(to_start_time, actions))

        last_time = None
        for current_time in start_times:
            if last_time is None:
                last_time = current_time
            elif last_time <= current_time:
                continue
            else:
                raise ValidationError(message="Action group is not sorted correctly by start time", reason=None)
