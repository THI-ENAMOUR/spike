from error.application_error import ApplicationError


class IllegalArgumentError(ApplicationError):
    """Thrown if the provided arguments to a callable are incorrect."""

    def __init__(self, message):
        super(IllegalArgumentError, self).__init__(message)
