import abc

from core.model.action.atomic.atomic_action import AtomicAction


class Controller(metaclass=abc.ABCMeta):
    """Controls the execution of a specific AtomicAction"""

    @abc.abstractmethod
    def execute_action(self, action: AtomicAction):
        raise NotImplementedError
