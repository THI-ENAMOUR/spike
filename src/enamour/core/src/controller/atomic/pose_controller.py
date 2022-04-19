from src.controller.controller import Controller
from src.core.model.action.atomic.pose_action import PoseAction
from src.core.model.action.timing_option import TimingOption
from src.exception.illegal_state_error import IllegalStateError
from src.exception.not_supported_error import NotSupportedError
from src.util.action_duration import ActionDuration


class PoseController(Controller):
    """Controller for moving the robot body in the provided position."""

    def execute_action(self, action: PoseAction, parent_duration: ActionDuration):
        if not isinstance(action, PoseAction):
            raise IllegalStateError(f"This controller does not support the action {action}")

        if action.timing_option == TimingOption.Always or action.timing_option == TimingOption.StartTime:
            # TODO: Jump instantly to position by publishing desired position
            pass
        elif action.timing_option == TimingOption.Duration:
            # TODO: Interpolate the pose based on the parent_duration, current location (TODO), and desired end position
            pass
        else:
            raise NotSupportedError(action)
