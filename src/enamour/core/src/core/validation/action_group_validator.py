from core.model.action.group.action_group import ActionGroup


class ActionGroupValidator:
    def validate(self, action_group: ActionGroup):
        """Validates an action group. Throws error if validation error occurred."""
        print(f"Validate action group {action_group.id}")
        # TODO: Validate stuff like start_time < end_time and contained action list is sorted by start_time
        pass
