import abc

from src.core.model.action.atomic.generic.atomic_action import AtomicAction


class Controller(metaclass=abc.ABCMeta):
    """Controls the execution of a specific AtomicAction"""

    @classmethod
    def __subclasshook__(cls, subclass):
        return (hasattr(subclass, "execute_action")) and callable(subclass.execute_action)

    @abc.abstractmethod
    def execute_action(self, action: AtomicAction):
        raise NotImplementedError
