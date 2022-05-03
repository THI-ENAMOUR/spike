from core.controller.controller import Controller
from core.model.action.atomic.pose_action import PoseAction
from core.model.action.timing_option import TimingOption
from error.illegal_state_error import IllegalStateError


class PoseController(Controller):
    """Controller for moving the robot body in the provided position."""

    def execute_action(self, action: "PoseAction"):

        if not isinstance(action, PoseAction):
            raise IllegalStateError(f"This controller does not support the action {action}")

        if action.timing_option == TimingOption.StartTime:
            # TODO Jump instantly to position by publishing desired position
            pass
        elif action.timing_option == TimingOption.Duration:
            # TODO Interpolate the pose based on the parent_duration, current location (TODO), and desired end position
            pass
        else:
            raise NotImplementedError(f"Timing option {action.timing_option} for action {action.id} not implemented")
