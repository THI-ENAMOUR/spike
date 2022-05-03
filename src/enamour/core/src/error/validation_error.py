import string

from error.application_error import ApplicationError


class ValidationError(ApplicationError):
    def __init__(self, reason: string, is_critical: bool = True, message: string = None):
        self.message = message if message is not None else f"Validation failed: {reason}"
        super().__init__(is_critical=is_critical)

    def __str__(self):
        return self.message
