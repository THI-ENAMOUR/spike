from controller import controller_provider
from controller.atomic.no_op_controller import NoOpController
from controller.atomic.pose_controller import PoseController
from core.completion_checker import check_completion_after_execution
from core.model.action.atomic.generic.action_execution_list import ActionExecutionList
from core.model.action.atomic.generic.atomic_action import AtomicAction
from core.validation.execution_list_validator import ExecutionListValidator


class ControllerOrganizer:
    """Retrieves the corresponding controller for an action and executes the action with it."""

    def __init__(
        self,
        validator: ExecutionListValidator = ExecutionListValidator(),
        movement_controller: PoseController = controller_provider.movement_controller,
        no_op_controller: NoOpController = controller_provider.no_op_controller,
    ):
        self.validator = validator
        self.movement_controller = movement_controller
        self.no_op_controller = no_op_controller

    def execute_actions(self, actions: ActionExecutionList):
        self.validator.validate(actions)
        for action in actions:
            self.__execute_action(action)

    @staticmethod
    def __execute_action(action: AtomicAction):
        print(f"Executing {action}")
        controller = action.get_controller()
        controller.execute_action(action)
        if check_completion_after_execution(action):
            action.complete()
