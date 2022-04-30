from src.core.model.action.group.action_group import ActionGroup


class ActionValidator:

    def validate(self, action_group: ActionGroup):
        """Validates an action group. Throws error if validation error occurred."""
        # TODO: Validate stuff like start_time < end_time and contained action list is sorted by start_time
        pass
