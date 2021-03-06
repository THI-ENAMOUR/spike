from error.application_error import ApplicationError


class IllegalStateError(ApplicationError):
    """Thrown if the point in code should not be reachable or the current situation violates the expected behaviour."""

    def __init__(self, message):
        super(IllegalStateError, self).__init__(message)
