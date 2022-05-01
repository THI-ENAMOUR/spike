from enamour.action_organizer.src.controller.display_controller import DisplayController
from enamour.action_organizer.src.controller.lambda_conrtoller import LambdaController
from enamour.action_organizer.src.controller.sound_controller import SoundController
from enamour.action_organizer.src.core.model.action.atomic.move_action import MoveAction
from enamour.action_organizer.src.core.model.action.atomic.stabilization_action import StabilizationAction
from src.controller.no_op_controller import NoOpController
from src.controller.pose_controller import PoseController

# List of controllers. Use these instead of creating new ones.

# TODO: Move this part inside the specific controller or inside the ControllerOrganizer, but it results in a circular
# TODO: dependency because of the needed import.

movement_controller: PoseController = PoseController()
no_op_controller: NoOpController = NoOpController()
display_controller = DisplayController()
lambda_controller = LambdaController()
sound_controller = SoundController()
stabilization_controller = StabilizationAction()
move_controller = MoveAction()
