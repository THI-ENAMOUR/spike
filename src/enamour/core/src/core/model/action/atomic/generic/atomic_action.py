from __future__ import annotations

import abc

# TODO: Cannot import directly, because of circular dependency. Better way?
# Source: https://stackoverflow.com/questions/39740632/python-type-hinting-without-cyclic-imports
from typing import TYPE_CHECKING, List, Callable, Set

from core.model.action.action import Action
from core.model.action.action_type import ActionType
from core.model.action.execution_method import ExecutionMethod
from core.model.action.timing_option import TimingOption

if TYPE_CHECKING:
    from controller.controller import Controller


class AtomicAction(Action, metaclass=abc.ABCMeta):
    """Action which is atomic. Each atomic action has a corresponding controller for their execution."""

    def __init__(self, action_type: ActionType, timing_option: TimingOption, execution_method: ExecutionMethod):
        super().__init__(timing_option=timing_option, execution_method=execution_method)
        self.action_type = action_type

    @abc.abstractmethod
    def get_controller(self) -> Controller:
        raise NotImplementedError

    @staticmethod
    def to_action_type_set(actions: List[AtomicAction]) -> Set[ActionType]:
        to_action_type: Callable[[AtomicAction], ActionType] = lambda a: a.action_type
        return set(map(to_action_type, actions))
