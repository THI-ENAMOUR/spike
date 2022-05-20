from util.json_mapper import to_json


class State:
    current_action_id = None
    action_queue = []
    latest_completed_actions = []
    latest_error_actions = []

    def __init__(self):
        self.current_action_id = State.current_action_id
        self.action_queue = State.action_queue
        self.latest_error_actions = State.latest_error_actions
        self.latest_completed_actions = State.latest_completed_actions

    @staticmethod
    def update(action_queue):
        action_queue.lock_queue_for_execution(func=State.set_attributes)

    @staticmethod
    def set_attributes(action_queue):
        current_action = action_queue.peek_and_pop_completed()
        State.current_action_id = current_action.id if current_action is not None else None
        State.action_queue = list(map(lambda action: str(action.id), action_queue.queue))
        State.latest_completed_actions = list(map(lambda action: str(action.id), action_queue.latest_completed_actions))
        State.latest_error_actions = list(map(lambda action: str(action.id), action_queue.latest_error_actions))

    @staticmethod
    def to_json():
        return to_json(State())
