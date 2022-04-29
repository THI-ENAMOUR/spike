from src.controller import controller_provider
from src.controller.atomic.no_op_controller import NoOpController
from src.controller.atomic.pose_controller import PoseController
from src.core.completion_checker import check_completion_after
from src.core.model.action.atomic.generic.action_execution_list import ActionExecutionList
from src.core.model.action.atomic.generic.atomic_action import AtomicAction
from src.core.model.action.execution_method import ExecutionMethod


class ControllerOrganizer:
    """Retrieves the corresponding controller for an action and executes the action with it."""

    def __init__(
        self,
        movement_controller: PoseController = controller_provider.movement_controller,
        no_op_controller: NoOpController = controller_provider.no_op_controller,
    ):
        self.movement_controller = movement_controller
        self.no_op_controller = no_op_controller

    def execute_actions(self, actions: ActionExecutionList):
        self.validate_actions(actions)
        for action in actions:
            self.execute_action(action)

    @staticmethod
    def validate_actions(actions: ActionExecutionList):
        action_types = set()
        for action in actions:
            # TODO: Think about if Pose and Navigation should have the same action type or not
            action_type = action.action_type if not action.action_type.is_movement_action() else "movement"

            if action.execution_method == ExecutionMethod.NO_SAME_TYPE and action_type in action_types:
                raise ValueError
            elif action.execution_method == ExecutionMethod.SOLO and len(actions) > 1:
                raise ValueError

            action_types.add(action_type)

    @staticmethod
    def execute_action(action: AtomicAction):
        print(f"Executing {action}")
        controller = action.get_controller()
        controller.execute_action(action)
        if check_completion_after(action):
            action.complete()
