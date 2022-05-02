import abc
from enum import Enum


class ApiActionType(Enum):
    SIT = "sit"
    GROUP = "group"


class ApiAction(metaclass=abc.ABCMeta):
    type: ApiActionType

    @staticmethod
    def from_json(data: dict):
        # TODO: Implement correctly and in a nice, readable way including error handling

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
        self.actions = actions if actions is not None else []

    @staticmethod
    def from_json(data: dict):
        actions: "list[ApiAction]" = []
        for action in data["actions"]:
            actions.append(ApiAction.from_json(action))

        return ApiActionGroup(actions=actions)


class ApiSitAction(ApiAction):
    type = ApiActionType.SIT

    @staticmethod
    def from_json(data: dict):
        return ApiSitAction()
