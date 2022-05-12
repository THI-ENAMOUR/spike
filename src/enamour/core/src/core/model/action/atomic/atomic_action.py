import abc

from core.model.action.action import Action


class AtomicAction(Action):
    """Action which is atomic. Each atomic action has a corresponding controller for their execution."""

    __metaclass__ = abc.ABCMeta

    def __init__(self, action_type, timing_option, execution_method):
        super(AtomicAction, self).__init__(timing_option=timing_option, execution_method=execution_method)
        self.action_type = action_type

    @abc.abstractmethod
    def get_controller(self):
        raise NotImplementedError

    @staticmethod
    def to_action_type_set(actions):
        return set(map(lambda a: a.action_type, actions))
