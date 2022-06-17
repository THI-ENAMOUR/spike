import abc
from enum import Enum

from core.model.action.atomic.display_action import DisplayAction
from core.model.action.atomic.head_action import HeadAction
from core.model.action.atomic.navigation_action import NavigationAction
from core.model.action.atomic.no_op_action import NoOpAction
from core.model.action.atomic.pose_action import PoseAction
from core.model.action.atomic.sound_action import SoundAction
from core.model.action.atomic.stabilization_action import StabilizationAction
from core.model.action.group.action_group import ActionGroup
from core.model.action.group.sit_action import SitAction
from error.deserialization_error import DeserializationError
from util.degree_converter import degree_to_radiant
from util.json_mapper import get, get_default


class ApiActionType(Enum):
    GROUP = "group"
    SIT = "sit"
    SOUND = "sound"
    DISPLAY = "display"
    NAVIGATION = "navigation"
    NOOP = "no_op"
    POSE = "pose"
    HEAD = "head"
    STABILIZATION = "stabilization"


class ApiAction(object):
    __metaclass__ = abc.ABCMeta

    type = None

    def __init__(self, start_ms, end_ms=None):
        self.start_ms = start_ms
        self.end_ms = end_ms

    @staticmethod
    def from_json(data):
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
            ApiHeadAction.type.value: lambda x: ApiHeadAction.from_json(x),
        }[type]

        if parse_action is None:
            raise DeserializationError(message="Key 'type' not contained in action")

        return parse_action(data)


class ApiActionGroup(ApiAction):
    type = ApiActionType.GROUP

    def __init__(self, start_ms, actions):
        super(ApiActionGroup, self).__init__(start_ms=start_ms)
        self.actions = actions if actions is not None else []

    @staticmethod
    def from_json(data):
        actions_json = get(data, "actions")

        actions = []
        for action in actions_json:
            actions.append(ApiAction.from_json(action))

        start_ms = get(data, "start_ms", expected_type=int)
        return ApiActionGroup(start_ms=start_ms, actions=actions)

    def to_action_group(self):
        actions = []
        for action in self.actions:
            actions.append(action.to_action_group())
        return ActionGroup(actions=actions, start_ms=self.start_ms)


class ApiSitAction(ApiAction):
    type = ApiActionType.SIT

    def __init__(self, start_ms):
        super(ApiSitAction, self).__init__(start_ms=start_ms)

    @staticmethod
    def from_json(data):
        start_ms = get(data, "start_ms", expected_type=int)
        return ApiSitAction(start_ms=start_ms)

    def to_action_group(self):
        return SitAction(self.start_ms)


class ApiSoundAction(ApiAction):
    type = ApiActionType.SOUND

    def __init__(self, start_ms, end_ms, data):
        super(ApiSoundAction, self).__init__(start_ms=start_ms, end_ms=end_ms)
        self.data = data

    @staticmethod
    def from_json(data):
        start_ms = get(data, "start_ms", expected_type=int)
        end_ms = get_default(data, "end_ms", default=None)
        sound_data = get_default(data, "data", default={})
        return ApiSoundAction(start_ms=start_ms, end_ms=end_ms, data=sound_data)

    def to_action_group(self):
        return SoundAction(start_ms=self.start_ms, end_ms=self.end_ms, data=self.data)


class ApiDisplayAction(ApiAction):
    type = ApiActionType.DISPLAY

    def __init__(self, start_ms, end_ms, data):
        super(ApiDisplayAction, self).__init__(start_ms=start_ms, end_ms=end_ms)
        self.data = data

    @staticmethod
    def from_json(data):
        start_ms = get(data, "start_ms", expected_type=int)
        end_ms = get_default(data, "end_ms", default=None)
        display_data = get_default(data, "data", default={})
        return ApiDisplayAction(start_ms=start_ms, end_ms=end_ms, data=display_data)

    def to_action_group(self):
        return DisplayAction(start_ms=self.start_ms, end_ms=self.end_ms, data=self.data)


class ApiNavigationAction(ApiAction):
    type = ApiActionType.NAVIGATION

    def __init__(self, start_ms, x, y, yaw, body_height):
        super(ApiNavigationAction, self).__init__(start_ms=start_ms)

        self.x = x
        self.y = y
        self.yaw = yaw
        self.body_height = body_height

    @staticmethod
    def from_json(data):
        start_ms = get(data, "start_ms", expected_type=int)
        x = get_default(data, "x", default=0)
        y = get_default(data, "y", default=0)
        yaw = degree_to_radiant(get_default(data, "yaw", default=None))
        body_height = get_default(data, "body_height", default=None)

        return ApiNavigationAction(start_ms=start_ms, x=x, y=y, yaw=yaw, body_height=body_height)

    def to_action_group(self):
        return NavigationAction(
            start_ms=self.start_ms, end_ms=self.end_ms, x=self.x, y=self.y, yaw=self.yaw, body_height=self.body_height
        )


class ApiNoOpAction(ApiAction):
    type = ApiActionType.NOOP

    def __init__(self, start_ms):
        super(ApiNoOpAction, self).__init__(start_ms=start_ms)

    @staticmethod
    def from_json(data):
        start_ms = get(data, "start_ms", expected_type=int)
        return ApiNoOpAction(start_ms=start_ms)

    def to_action_group(self):
        return NoOpAction(start_ms=self.start_ms)


class ApiPoseAction(ApiAction):
    type = ApiActionType.POSE

    def __init__(self, start_ms, roll, pitch, yaw, body_height):
        super(ApiPoseAction, self).__init__(start_ms=start_ms)
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.body_height = body_height

    @staticmethod
    def from_json(data):
        start_ms = get(data, "start_ms", expected_type=int)
        roll = degree_to_radiant(get_default(data, "roll", default=None))
        pitch = degree_to_radiant(get_default(data, "pitch", default=None))
        yaw = degree_to_radiant(get_default(data, "yaw", default=None))
        body_height = get_default(data, "body_height", default=None)

        return ApiPoseAction(start_ms=start_ms, roll=roll, pitch=pitch, yaw=yaw, body_height=body_height)

    def to_action_group(self):
        return PoseAction(
            start_ms=self.start_ms, roll=self.roll, pitch=self.pitch, yaw=self.yaw, body_height=self.body_height
        )


class ApiHeadAction(ApiAction):
    type = ApiActionType.HEAD

    def __init__(self, start_ms, roll, pitch, yaw):
        super(ApiHeadAction, self).__init__(start_ms=start_ms)
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    @staticmethod
    def from_json(data):
        start_ms = get(data, "start_ms", expected_type=int)
        roll = degree_to_radiant(get_default(data, "roll", default=None))
        pitch = degree_to_radiant(get_default(data, "pitch", default=None))
        yaw = degree_to_radiant(get_default(data, "yaw", default=None))

        return ApiHeadAction(start_ms=start_ms, roll=roll, pitch=pitch, yaw=yaw)

    def to_action_group(self):
        return HeadAction(start_ms=self.start_ms, roll=self.roll, pitch=self.pitch, yaw=self.yaw)


class ApiStabilizationAction(ApiAction):
    type = ApiActionType.STABILIZATION

    def __init__(self, start_ms):
        super(ApiStabilizationAction, self).__init__(start_ms=start_ms)

    @staticmethod
    def from_json(data):
        start_ms = get(data, "start_ms", expected_type=int)
        return ApiStabilizationAction(start_ms=start_ms)

    def to_action_group(self):
        return StabilizationAction(start_ms=self.start_ms)
