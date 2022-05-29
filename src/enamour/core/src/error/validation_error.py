from error.application_error import ApplicationError


class ValidationError(ApplicationError):
    """Thrown if a validation failed."""

    def __init__(self, reason, is_critical=True, message=None):
        message = message if message is not None else "Validation failed: {reason}".format(reason=reason)
        super(ValidationError, self).__init__(message=message, is_critical=is_critical)
