import abc
from typing import TYPE_CHECKING, List

from src.util.action_duration import ActionDuration

if TYPE_CHECKING:
    from src.core.model.action.action import Action


class TimingOption(metaclass=abc.ABCMeta):
    # If you add new TimingOptions, don't forget to add the assignment at the end of this file
    StartTime = None
    Duration = None

    @abc.abstractmethod
    def in_time_frame(self, action: "Action", time: ActionDuration) -> bool:
        raise NotImplementedError

    @staticmethod
    def sort(actions: List["Action"]):
        """Uses list sort function, which is in-place and stable"""
        actions.sort(key=TimingOption.__get_start_time)

    @staticmethod
    def __get_start_time(elem: "Action"):
        if isinstance(elem.timing_option, StartTime):
            return elem.timing_option.start_time
        elif isinstance(elem.timing_option, Duration):
            return elem.timing_option.start_time
        else:
            raise NotImplementedError


class StartTime(TimingOption):
    def __init__(self, start_time: ActionDuration):
        super().__init__()
        self.start_time = start_time

    def in_time_frame(self, action: "Action", time: ActionDuration = None) -> bool:
        """Returns true if the start time < current time. The start time is exclusive.
        If both of them are zero, true is also returned."""
        return not action.completed and (self.start_time < time or (self.start_time.is_zero() and time.is_zero()))

    def __str__(self):
        return f"{self.__class__.__name__}(start: {self.start_time}ns)"

    def __eq__(self, other):
        # "other is StartTime" allows an enum-like equality check: TimingOptions.StartTime == self return true
        return (isinstance(other, StartTime) and self.start_time == other.start_time) or other is StartTime

    def __ne__(self, other):
        return not self.__eq__(other)


class Duration(TimingOption):
    def __init__(self, start_time: ActionDuration, end_time: ActionDuration):
        super().__init__()
        if start_time > end_time:
            raise ValueError
        self.start_time = start_time
        self.end_time = end_time

    @classmethod
    def from_ms(cls, start: int, end: int) -> "Duration":
        return cls(start_time=ActionDuration(ms=start), end_time=ActionDuration(ms=end))

    def in_time_frame(self, action: "Action", time: ActionDuration = None) -> bool:
        """Returns true if the start time < current time <= end time. The start time is exclusive.
        If the start and current time are zero, true is also returned."""
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


TimingOption.StartTime = StartTime
TimingOption.Duration = Duration
