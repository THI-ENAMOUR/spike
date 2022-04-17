from __future__ import annotations

import abc

from src.core.model.action.action import Action
from src.core.model.action.action_type import ActionType

# TODO: Cannot import directly, because of circular dependency. Better way?
# Source: https://stackoverflow.com/questions/39740632/python-type-hinting-without-cyclic-imports
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from src.controller.controller import Controller


class AtomicAction(Action, metaclass=abc.ABCMeta):
    """Action which is atomic. Each atomic action has a corresponding controller for their execution."""

    def __init__(self, action_type: ActionType, selection_type, execution_method):
        super().__init__(selection_type=selection_type, execution_method=execution_method)
        self.action_type = action_type

    @abc.abstractmethod
    def get_controller(self) -> Controller:
        raise NotImplementedError
