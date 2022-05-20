import abc
from enum import Enum

from core.model.action.atomic.display_action import DisplayAction
from core.model.action.atomic.navigation_action import NavigationAction
from core.model.action.atomic.no_op_action import NoOpAction
from core.model.action.atomic.pose_action import PoseAction
from core.model.action.atomic.sound_action import SoundAction
from core.model.action.atomic.stabilization_action import StabilizationAction
from core.model.action.group.action_group import ActionGroup
from core.model.action.group.sit_action import SitAction
from error.deserialization_error import DeserializationError
from util.json_mapper import get


class ApiActionType(Enum):
    GROUP = "group"
    SIT = "sit"
    SOUND = "sound"
    DISPLAY = "display"
    NAVIGATION = "navigation"
    NOOP = "no_op"
    POSE = "pose"
    STABILIZATION = "stabilization"


class ApiAction(object):
    __metaclass__ = abc.ABCMeta

    type = None

    def __init__(self, start_time):
        self.start_time = start_time

    @staticmethod
    def from_json(data):
        # TODO: Implement correctly and in a nice, readable way including error handling

        type = get(data, "type")
        parse_action = {
            ApiActionGroup.type.value: lambda x: ApiActionGroup.from_json(x),
            ApiSitAction.type.value: lambda x: ApiSitAction.from_json(x),
            ApiSoundAction.type.value: lambda x: ApiSoundAction.from_json(x),
            ApiDisplayAction.type.value: lambda x: ApiDisplayAction.from_json(x),
            ApiNavigationAction.type.value: lambda x: ApiNavigationAction.from_json(x),
            ApiNoOpAction.type.value: lambda x: ApiNoOpAction.from_json(x),
            ApiPoseAction.type.value: lambda x: ApiPoseAction.from_json(x),
            ApiStabilizationAction.type.value: lambda x: ApiStabilizationAction.from_json(x),
        }[type]

        if parse_action is None:
            raise DeserializationError(message="Key 'type' not contained in action")

        return parse_action(data)


class ApiActionGroup(ApiAction):
    type = ApiActionType.GROUP

    def __init__(self, start_time, actions):
        super(ApiActionGroup, self).__init__(start_time=start_time)
        self.actions = actions if actions is not None else []

    @staticmethod
    def from_json(data):
        actions_json = get(data, "actions")

        actions = []
        for action in actions_json:
            actions.append(ApiAction.from_json(action))

        start_time = get(data, "start_time", expected_type=int)
        return ApiActionGroup(start_time=start_time, actions=actions)

    def to_action_group(self):
        actions = []
        for action in self.actions:
            actions.append(action.to_action_group())
        return ActionGroup(actions=actions, start_ms=self.start_time)


class ApiSitAction(ApiAction):
    type = ApiActionType.SIT

    def __init__(self, start_time):
        super(ApiSitAction, self).__init__(start_time=start_time)

    @staticmethod
    def from_json(data):
        start_time = get(data, "start_time", expected_type=int)
        return ApiSitAction(start_time=start_time)

    def to_action_group(self):
        return SitAction(self.start_time)


class ApiSoundAction(ApiAction):
    type = ApiActionType.SOUND

    def __init__(self, start_time):
        super(ApiSoundAction, self).__init__(start_time=start_time)

    @staticmethod
    def from_json(data):
        start_time = get(data, "start_time", expected_type=int)
        return ApiSoundAction(start_time=start_time)

    def to_action_group(self):
        return SoundAction(start_ms=self.start_time)


class ApiDisplayAction(ApiAction):
    type = ApiActionType.DISPLAY

    def __init__(self, start_time):
        super(ApiDisplayAction, self).__init__(start_time=start_time)

    @staticmethod
    def from_json(data):
        start_time = get(data, "start_time", expected_type=int)
        return ApiDisplayAction(start_time=start_time)

    def to_action_group(self):
        return DisplayAction(start_ms=self.start_time)


class ApiNavigationAction(ApiAction):
    type = ApiActionType.NAVIGATION

    def __init__(self, start_time):
        super(ApiNavigationAction, self).__init__(start_time=start_time)

    @staticmethod
    def from_json(data):
        start_time = get(data, "start_time", expected_type=int)
        return ApiNavigationAction(start_time=start_time)

    def to_action_group(self):
        return NavigationAction(start_ms=self.start_time)


class ApiNoOpAction(ApiAction):
    type = ApiActionType.NOOP

    def __init__(self, start_time):
        super(ApiNoOpAction, self).__init__(start_time=start_time)

    @staticmethod
    def from_json(data):
        start_time = get(data, "start_time", expected_type=int)
        return ApiNoOpAction(start_time=start_time)

    def to_action_group(self):
        return NoOpAction(start_time_ms=self.start_time)


class ApiPoseAction(ApiAction):
    type = ApiActionType.POSE

    def __init__(self, start_time):
        super(ApiPoseAction, self).__init__(start_time=start_time)

    @staticmethod
    def from_json(data):
        start_time = get(data, "start_time", expected_type=int)
        return ApiPoseAction(start_time=start_time)

    def to_action_group(self):
        return PoseAction(start_ms=self.start_time)


class ApiStabilizationAction(ApiAction):
    type = ApiActionType.STABILIZATION

    def __init__(self, start_time):
        super(ApiStabilizationAction, self).__init__(start_time=start_time)

    @staticmethod
    def from_json(data):
        start_time = get(data, "start_time", expected_type=int)
        return ApiStabilizationAction(start_time=start_time)

    def to_action_group(self):
        return StabilizationAction(start_ms=self.start_time)
