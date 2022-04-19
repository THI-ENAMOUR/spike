import abc
from typing import TYPE_CHECKING

from src.util.action_duration import ActionDuration

if TYPE_CHECKING:
    from src.core.model.action.action import Action


class TimingOption(metaclass=abc.ABCMeta):
    # If you add new TimingOptions, don't forget to add the assignment at the end of this file
    Always = None
    StartTime = None
    Duration = None

    @abc.abstractmethod
    def is_selected_action(self, action: "Action", time: ActionDuration) -> bool:
        raise NotImplementedError


class Always(TimingOption):
    def is_selected_action(self, action: "Action", time: ActionDuration = None) -> bool:
        return not action.completed

    def __str__(self):
        return f"{self.__class__.__name__}"

    def __eq__(self, other):
        # 'other is Always' returns true for 'timing == Always', so we don't need isinstance.
        return isinstance(other, Always) or other is Always

    def __ne__(self, other):
        return not self.__eq__(other)


class StartTime(TimingOption):
    def __init__(self, start_time: ActionDuration):
        super().__init__()
        self.start_time = start_time

    def is_selected_action(self, action: "Action", time: ActionDuration = None) -> bool:
        return not action.completed and self.start_time <= time

    def __str__(self):
        return f"{self.__class__.__name__}(start: {self.start_time}ns)"

    def __eq__(self, other):
        # 'other is StartTime' returns true for 'timing == StartTime', so we don't need isinstance.
        return (isinstance(other, StartTime) and self.start_time == other.start_time) or other is StartTime

    def __ne__(self, other):
        return not self.__eq__(other)


class Duration(TimingOption):
    def __init__(self, start_time: ActionDuration, end_time: ActionDuration):
        super().__init__()
        self.start_time = start_time
        self.end_time = end_time

    @classmethod
    def from_ms(cls, start: int, end: int) -> "Duration":
        return cls(start_time=ActionDuration(ms=start), end_time=ActionDuration(ms=end))

    def is_selected_action(self, action: "Action", time: ActionDuration = None) -> bool:
        return not action.completed and self.start_time <= time <= self.end_time

    def __str__(self):
        return f"{self.__class__.__name__}(start: {self.start_time}ns, end: {self.end_time}ns)"

    def __eq__(self, other):
        # 'other is Duration' returns true for 'timing == Duration', so we don't need isinstance.
        return other is Duration or (
            isinstance(other, Duration) and self.start_time == other.start_time and self.end_time == other.end_time
        )

    def __ne__(self, other):
        return not self.__eq__(other)


TimingOption.Always = Always
TimingOption.StartTime = StartTime
TimingOption.Duration = Duration
