"""

from __future__ import annotations

import abc

from src.util.action_duration import ActionDuration

# TODO: Cannot import directly, because of circular dependency. Better way?
# Source: https://stackoverflow.com/questions/39740632/python-type-hinting-without-cyclic-imports
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from src.core.model.action.action import Action


class SelectionType(metaclass=abc.ABCMeta):
    @abc.abstractmethod
    def is_selected_action(self, action: Action, time: ActionDuration) -> bool:
        raise NotImplementedError


class SelectAlways(SelectionType):
    def is_selected_action(self, action: Action, time: ActionDuration = None) -> bool:
        return not action.completed

    def __str__(self):
        return f'{self.__class__.__name__}'


class SelectStartTime(SelectionType):
    def __init__(self, start_time: ActionDuration, selection_type):
        super().__init__()
        self.start_time = start_time
        self.selection_type = selection_type

    def is_selected_action(self, action: Action, time: ActionDuration = None) -> bool:
        return not action.completed and self.start_time <= time

    def __str__(self):
        return f'{self.__class__.__name__}(start: {self.start_time}ns)'


class SelectDuration(SelectionType):
    def __init__(self, start_time: ActionDuration, end_time: ActionDuration):
        super().__init__()
        self.start_time = start_time
        self.end_time = end_time

    @classmethod
    def from_ms(cls, start: int, end: int) -> SelectDuration:
        return cls(start_time=ActionDuration(ms=start), end_time=ActionDuration(ms=end))

    def is_selected_action(self, action: Action, time: ActionDuration = None) -> bool:
        return not action.completed and self.start_time <= time <= self.end_time

    def __str__(self):
        return f'{self.__class__.__name__}(start: {self.start_time}ns, end: {self.end_time}ns)'

"""
