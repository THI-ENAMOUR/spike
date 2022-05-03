from error.application_error import ApplicationError


class IllegalArgumentError(ApplicationError):
    """Thrown if the provided arguments to a callable are incorrect."""

    def __init__(self, message):
        self.message = message

    def __str__(self):
        return self.message
