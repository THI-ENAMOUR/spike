from src.core.model.action.action import Action
from src.core.model.action.timing_option import TimingOption
from src.util.action_duration import ActionDuration


# Checks if the action is completed. The functions are called in different parts of the applications.
# Move this logic inside a class or the actions themselves once it will be more complicated.
# Actions can be completed before they get selected, by the controller themselves or after they get executed.


def check_completion_before(action: Action, time: ActionDuration) -> bool:
    return isinstance(action.timing_option, TimingOption.Duration) and action.timing_option.end_time < time


# Currently, we finish actions all actions instantly if they are not durations.
# If the amount of actions that are an exception to this are too high,
# relocate this logic inside every specific controller.
def check_completion_after(action: Action) -> bool:
    return not isinstance(action.timing_option, TimingOption.Duration)
