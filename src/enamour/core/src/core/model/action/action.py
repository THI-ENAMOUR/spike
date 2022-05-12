import abc
import uuid

from error.illegal_state_error import IllegalStateError


class Action(object):
    """Most generic, abstract action. Containing information all actions share among themselves."""

    __metaclass__ = abc.ABCMeta

    def __init__(self, timing_option, execution_method, parent=None):
        self.id = uuid.uuid4()
        self.completed = False
        self.timing_option = timing_option
        self.execution_method = execution_method
        self.parent = parent

    def complete(self):
        self.completed = True

    def in_time_frame(self, time_stamp):
        """Returns true if the time stamp is within the timing option of the action is"""
        return self.timing_option.in_time_frame(self, time_stamp)

    def get_parent_time(self):
        if self.parent is not None:
            return self.parent.time
        raise IllegalStateError("Could not get parent time, since parent is none.")

    def __str__(self):
        return "{class_name}(id: {id}, timing: {timing}, execution: {execution})".format(
            class_name=self.__class__.__name__,
            id=self.id,
            timing=self.timing_option,
            execution=self.execution_method.value,
        )
