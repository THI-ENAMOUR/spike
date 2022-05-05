import abc
from typing import TYPE_CHECKING, Callable

from core.model.common.time_stamp import TimeStamp
from error.illegal_argument_error import IllegalArgumentError

if TYPE_CHECKING:
    from core.model.action.action import Action, ActionList


class TimingOption(metaclass=abc.ABCMeta):
    """Determines in what way the action is selected/timed within an action group."""

    @abc.abstractmethod
    def in_time_frame(self, action: "Action", time: TimeStamp) -> bool:
        """Returns true if the timing option is within a certain time stamp."""
        raise NotImplementedError

    @staticmethod
    def sort(
        actions: "ActionList",
        sort_function: Callable[["Action"], TimeStamp] = lambda x: x.timing_option.get_start_time(),
    ):
        """In-place and stable sorts actions by their start date"""
        actions.sort(key=sort_function)

    @abc.abstractmethod
    def get_start_time(self):
        """Returns the start time stamp of the timing option"""
        raise NotImplementedError


class StartTime(TimingOption):
    def __init__(self, start_ms: int):
        super().__init__()
        self.start_time = TimeStamp(start_ms)

    def in_time_frame(self, action: "Action", time: TimeStamp = None) -> bool:
        """Returns true if the start_time < time. The start time is exclusive.
        If both of them are zero returns true."""
        return not action.completed and (self.start_time < time or (self.start_time.is_zero() and time.is_zero()))

    def get_start_time(self):
        return self.start_time

    def __str__(self):
        return f"{self.__class__.__name__}(start: {self.start_time}ns)"

    def __eq__(self, other):
        # "other is StartTime" allows an enum-like equality check: TimingOptions.StartTime == self return true
        return (isinstance(other, StartTime) and self.start_time == other.start_time) or other is StartTime

    def __ne__(self, other):
        return not self.__eq__(other)


class Duration(TimingOption):
    def __init__(self, start_ms: int, end_ms: int):
        super().__init__()
        if start_ms > end_ms:
            raise IllegalArgumentError(f"Start time {start_ms} should be smaller than end time {end_ms}")
        self.start_time = TimeStamp(ms=start_ms)
        self.end_time = TimeStamp(ms=end_ms)

    def in_time_frame(self, action: "Action", time: TimeStamp = None) -> bool:
        """Returns true if the start_time < time <= end_time. The start time is exclusive.
        If the start_time and time are zero returns true"""
        return not action.completed and (
            (self.start_time < time and self.start_time <= self.end_time)
            or (self.start_time.is_zero() and time.is_zero())
        )

    def get_start_time(self):
        return self.start_time

    def __str__(self):
        return f"{self.__class__.__name__}(start: {self.start_time}ns, end: {self.end_time}ns)"

    def __eq__(self, other):
        # "other is Duration" allows an enum-like equality check: TimingOptions.Duration == self return true
        return other is Duration or (
            isinstance(other, Duration) and self.start_time == other.start_time and self.end_time == other.end_time
        )

    def __ne__(self, other):
        return not self.__eq__(other)
