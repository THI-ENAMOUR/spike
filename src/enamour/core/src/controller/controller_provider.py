from src.controller.atomic.no_op_controller import NoOpController
from src.controller.atomic.pose_controller import PoseController

# List of controllers. Use these instead of creating new ones.

movement_controller: PoseController = PoseController()
no_op_controller: NoOpController = NoOpController()
