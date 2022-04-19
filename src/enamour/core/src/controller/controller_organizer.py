from src.controller import controller_provider
from src.controller.atomic.no_op_controller import NoOpController
from src.controller.atomic.pose_controller import PoseController
from src.core.completion_checker import check_completion_after
from src.core.model.action.atomic.generic.action_execution_list import ActionExecutionList
from src.core.model.action.atomic.generic.atomic_action import AtomicAction
from src.util.action_duration import ActionDuration


class ControllerOrganizer:
    """Retrieves the corresponding controller for an action and executes the action with it."""

    def __init__(
        self,
        movement_controller: PoseController = controller_provider.movement_controller,
        no_op_controller: NoOpController = controller_provider.no_op_controller,
    ):
        self.movement_controller = movement_controller
        self.no_op_controller = no_op_controller

    def execute_actions(self, actions: ActionExecutionList, parent_duration: ActionDuration):
        for action in actions:
            self.execute_action(action, parent_duration)

    @staticmethod
    def execute_action(action: AtomicAction, parent_duration: ActionDuration):
        print(f"Executing {action}")
        controller = action.get_controller()
        controller.execute_action(action, parent_duration)
        if check_completion_after(action):
            action.complete()
