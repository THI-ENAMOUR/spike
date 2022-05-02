from typing import TYPE_CHECKING

from exception.application_error import ApplicationError

if TYPE_CHECKING:
    from core.model.action.action import Action


class NotSupportedError(ApplicationError):
    """Thrown if the specific action / feature is not yet or will not be supported."""

    def __init__(self, message="This action not supported"):
        self.message = message
        super(ApplicationError, self).__init__(self.message)

    @classmethod
    def from_action(cls, action: "Action"):
        return cls(message=f"The action {action} is not supported")

    def __str__(self):
        return f"{self.message}"
