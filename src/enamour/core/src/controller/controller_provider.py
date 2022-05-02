from controller.atomic.no_op_controller import NoOpController
from controller.atomic.pose_controller import PoseController

# List of controllers. Use these instead of creating new ones.
# This file is separated in order to reduce the number of cyclic imports

movement_controller: PoseController = PoseController()
no_op_controller: NoOpController = NoOpController()
