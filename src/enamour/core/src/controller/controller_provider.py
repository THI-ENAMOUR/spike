from src.controller.no_op_controller import NoOpController
from src.controller.pose_controller import PoseController

# List of controllers. Use these instead of creating new ones.

# TODO: Move this part inside the specific controller or inside the ControllerOrganizer, but it results in a circular
# TODO: dependency because of the needed import.

movement_controller: PoseController = PoseController()
no_op_controller: NoOpController = NoOpController()
