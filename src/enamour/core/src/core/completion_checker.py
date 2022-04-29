from src.core.model.action.action import Action
from src.core.model.action.group.action_group import ActionGroup
from src.core.model.action.timing_option import TimingOption


# Checks if the action is completed. The functions are called in different parts of the applications.
# Move this logic inside a class or the actions themselves once it will be more complicated.
# Actions can be completed before they get selected, by the controller themselves or after they get executed.


def check_completion_before(action: Action) -> bool:
    return (
        not isinstance(action, ActionGroup)
        and isinstance(action.timing_option, TimingOption.Duration)
        and action.timing_option.end_time < action.get_parent_time()
    )


# Currently, we finish actions all actions instantly if they are not durations.
# If the amount of actions that are an exception to this are too high,
# relocate this logic inside every specific controller.
def check_completion_after(action: Action) -> bool:
    return not isinstance(action, ActionGroup) and not isinstance(action.timing_option, TimingOption.Duration)
