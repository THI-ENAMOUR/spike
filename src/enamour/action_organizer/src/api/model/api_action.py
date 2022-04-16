import abc
from enum import Enum


class ApiActionType(Enum):
    SIT = "sit"
    GROUP = "group"


class ApiAction(metaclass=abc.ABCMeta):
    type: ApiActionType

    @classmethod
    def __subclasshook__(cls, subclass):
        return hasattr(subclass, "type")

    @staticmethod
    def from_json(data: dict):
        parse_action = {
            ApiSitAction.type.value: lambda x: ApiSitAction.from_json(x),
            ApiActionGroup.type.value: lambda x: ApiActionGroup.from_json(x),
        }[data["type"]]

        if parse_action is None:
            raise KeyError

        return parse_action(data)


class ApiActionGroup(ApiAction):
    type = ApiActionType.GROUP

    def __init__(self, actions: "list[ApiAction]" = None):
        if actions is None:
            actions = []

        self.actions = actions

    @staticmethod
    def from_json(data: dict):
        actions: "list[ApiAction]" = []
        for action in data["actions"]:
            actions.append(ApiAction.from_json(action))

        return ApiActionGroup(actions=actions)


class ApiSitAction(ApiAction):
    type = ApiActionType.SIT

    def __init__(self):
        self.type = type

    @staticmethod
    def from_json(data: dict):
        return ApiSitAction()
