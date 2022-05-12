from core.completion_checker import check_completion_after_execution
from core.controller.atomic.display_controller import DisplayController
from core.controller.atomic.lambda_conrtoller import LambdaController
from core.controller.atomic.navigation_controller import NavigationController
from core.controller.atomic.no_op_controller import NoOpController
from core.controller.atomic.pose_controller import PoseController
from core.controller.atomic.sound_controller import SoundController
from core.controller.atomic.stabilization_controller import StabilizationController
from core.validation.execution_list_validator import ExecutionListValidator
from util.logger import Logger


class ControllerOrganizer:
    """Retrieves the corresponding controller for an action and executes the action with it."""

    __logger = Logger(__name__)

    pose_controller = PoseController()
    navigation_controller = NavigationController()
    no_op_controller = NoOpController()
    lambda_controller = LambdaController()
    display_controller = DisplayController()
    sound_controller = SoundController()
    stabilization_controller = StabilizationController()

    def __init__(self, validator=ExecutionListValidator()):
        self.validator = validator

    def execute_actions(self, actions):
        self.validator.validate(actions)
        for action in actions:
            self.__execute_action(action)

    def __execute_action(self, action):
        self.__logger.info("Executing " + str(action))
        controller = action.get_controller()
        controller.execute_action(action)
        if check_completion_after_execution(action):
            action.complete()
