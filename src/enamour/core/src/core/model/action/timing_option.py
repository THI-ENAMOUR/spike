import abc
from typing import TYPE_CHECKING

from core.model.common.action_duration import ActionDuration
from error.illegal_argument_error import IllegalArgumentError
from error.illegal_state_error import IllegalStateError

if TYPE_CHECKING:
    from core.model.action.action import Action, ActionList


class TimingOption(metaclass=abc.ABCMeta):
    # If you add new TimingOptions, don't forget to add the assignment at the end of this file
    StartTime = None
    Duration = None

    @abc.abstractmethod
    def in_time_frame(self, action: "Action", time: ActionDuration) -> bool:
        raise NotImplementedError

    @staticmethod
    def sort(actions: "ActionList"):
        """In-place and stable sorts actions by their start date"""
        actions.sort(key=TimingOption.__get_start_time)

    @staticmethod
    def __get_start_time(action: "Action"):
        if isinstance(action.timing_option, StartTime):
            return action.timing_option.start_time
        elif isinstance(action.timing_option, Duration):
            return action.timing_option.start_time
        else:
            raise IllegalStateError(f"The timeing option of action {action.id} is an unknown instance type")


class StartTime(TimingOption):
    def __init__(self, start_time: ActionDuration):
        super().__init__()
        self.start_time = start_time

    def in_time_frame(self, action: "Action", time: ActionDuration = None) -> bool:
        """Returns true if the start_time < time. The start time is exclusive.
        If both of them are zero returns true."""
        return not action.completed and (self.start_time < time or (self.start_time.is_zero() and time.is_zero()))

    def __str__(self):
        return f"{self.__class__.__name__}(start: {self.start_time}ns)"

    def __eq__(self, other):
        # "other is StartTime" allows an enum-like equality check: TimingOptions.StartTime == self return true
        return (isinstance(other, StartTime) and self.start_time == other.start_time) or other is StartTime

    def __ne__(self, other):
        return not self.__eq__(other)


TimingOption.StartTime = StartTime


class Duration(TimingOption):
    def __init__(self, start_time: ActionDuration, end_time: ActionDuration):
        super().__init__()
        if start_time > end_time:
            raise IllegalArgumentError(f"Start time {start_time} should be smaller than end time {end_time}")
        self.start_time = start_time
        self.end_time = end_time

    @classmethod
    def from_ms(cls, start: int, end: int) -> "Duration":
        return cls(start_time=ActionDuration(ms=start), end_time=ActionDuration(ms=end))

    def in_time_frame(self, action: "Action", time: ActionDuration = None) -> bool:
        """Returns true if the start_time < time <= end_time. The start time is exclusive.
        If the start_time and time are zero returns true"""
        return not action.completed and (
            (self.start_time < time and self.start_time <= self.end_time)
            or (self.start_time.is_zero() and time.is_zero())
        )

    def __str__(self):
        return f"{self.__class__.__name__}(start: {self.start_time}ns, end: {self.end_time}ns)"

    def __eq__(self, other):
        # "other is Duration" allows an enum-like equality check: TimingOptions.Duration == self return true
        return other is Duration or (
            isinstance(other, Duration) and self.start_time == other.start_time and self.end_time == other.end_time
        )

    def __ne__(self, other):
        return not self.__eq__(other)


TimingOption.Duration = Duration
