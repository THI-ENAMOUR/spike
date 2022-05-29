import math

from core.model.action.action_type import ActionType
from core.model.action.atomic.atomic_action import AtomicAction
from core.model.action.execution_method import ExecutionMethod
from core.model.action.timing_option import StartTime, Duration

from error.illegal_argument_error import IllegalArgumentError


class PoseAction(AtomicAction):



    def __init__(self, start_ms, end_ms=None, timing_option=None, roll=0, pitch=0, yaw=0):

        if timing_option is None:
            if end_ms is not None:
                timing_option = Duration(start_ms=start_ms, end_ms=end_ms)
            else:
                timing_option = StartTime(start_ms=start_ms)

        super(PoseAction, self).__init__(
            action_type=ActionType.BODY_MOVEMENT_ACTION,
            timing_option=timing_option,
            execution_method=ExecutionMethod.NO_SAME_TYPE,
        )

        PoseAction.validate_angle(roll, "roll")
        self.roll = roll
        PoseAction.validate_angle(pitch, "pitch")
        self.pitch = pitch
        PoseAction.validate_angle(yaw, "yaw")
        self.yaw = yaw

    MAX_ANGLE = 2 * math.pi
    MIN_ANGLE = -MAX_ANGLE

    @staticmethod
    def validate_angle(angle, name):
        if angle > PoseAction.MAX_ANGLE or angle < PoseAction.MIN_ANGLE:
            raise IllegalArgumentError("Angle {name} does not have valid value: {value}".format(name=name, value=angle))

    def get_controller(self):
        # Local import to break cyclic import chain
        from core.controller.controller_organizer import ControllerOrganizer

        return ControllerOrganizer.pose_controller
