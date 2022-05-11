import json
import this

from matplotlib.font_manager import json_dump

class State:
    current_action = None
    action_queue = []
    latest_completed_actions = []
    latest_error_actions = []

    @staticmethod
    def update(action_queue):
        action_queue.lock_queue_for_exectution(func = State.set_attributs)
    
    @staticmethod
    def set_attributes(action_queue):
        State.current_action = action_queue.peek_and_pop_completed()
        State.action_queue = list(map(lambda action: str(action.id), action_queue.queue))
        State.latest_completed_actions = list(map(lambda action: str(action.id), action_queue.latest_completed_actions))
        State.latest_error_actions = list(map(lambda action: str(action.id), action_queue.latest_error_actions))
    
    @staticmethod
    def to_json():
        return json.dumps(this.__dict__)
