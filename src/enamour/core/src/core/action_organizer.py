import rospy

from controller.controller_organizer import ControllerOrganizer
from core.action_queue import ActionQueue
from core.event_bus import event_bus
from core.model.common.action_duration import ActionDuration
from exception.illegal_state_error import IllegalStateError
from util.config import Config
from util.synchronized import synchronized


class ActionOrganizer:
    """Organizes the selection and execution of an ActionQueue"""

    def __init__(
        self,
        controller_organizer: ControllerOrganizer = ControllerOrganizer(),
        action_queue: ActionQueue = ActionQueue(),
    ):
        self.controller_organizer = controller_organizer
        self.action_queue = action_queue

        self.loop_rate = rospy.Rate(Config.loop_rate)
        self.last_loop_duration = ActionDuration(ns=0)
        self.running = False

    @synchronized
    def start(self):
        """Start an infinite loop of retrieving and executing the next action."""
        self.running = True
        while event_bus.is_running and self.running:
            self.execute_action()
            # Sleep for a set duration to maintain a steady tick rate
            self.loop_rate.sleep()

    def execute_action(self):
        """Executes the next action in the action queue."""

        # TODO: Implement correctly

        current_action_group = self.action_queue.peek_and_pop_completed()

        if current_action_group is None:
            # Queue is empty, do nothing
            return

        next_actions = current_action_group.get_next_actions()

        if len(next_actions) == 0:
            if current_action_group.completed:
                # Current action is completed so no action will be performed, therefore run this method again to
                # receive the next action in the queue.
                self.execute_action()
            else:
                raise IllegalStateError("No next action returned even tho action group is not completed")

        self.controller_organizer.execute_actions(next_actions)

        current_action_group.update_parent_time_of_executed_actions(self.loop_rate.sleep_dur)

        print("----------------------------------------------------------------------------------")
