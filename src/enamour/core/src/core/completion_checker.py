from core.model.action.action import Action
from core.model.action.group.action_group import ActionGroup
from core.model.action.timing_option import TimingOption


# Checks if the action is completed. The functions are called in different parts of the applications.
# Move this logic inside a class or the actions themselves once it will be more complicated.
# Actions can be completed before they get selected, by the controller themselves, or after they got executed.


def check_completion_before_selection(action: Action) -> bool:
    return (
        not isinstance(action, ActionGroup)
        and isinstance(action.timing_option, TimingOption.Duration)
        and action.timing_option.end_time < action.get_parent_time()
    )


def check_completion_after_execution(action: Action) -> bool:
    # Currently, we finish actions all actions instantly if they are not durations.
    return not isinstance(action, ActionGroup) and not isinstance(action.timing_option, TimingOption.Duration)
