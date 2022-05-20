import abc


class Controller(object):
    """Controls the execution of a specific AtomicAction"""

    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def execute_action(self, action):
        raise NotImplementedError
