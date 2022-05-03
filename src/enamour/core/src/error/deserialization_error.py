import string

from error.validation_error import ValidationError


class DeserializationError(ValidationError):
    def __init__(self, message: string):
        self.message = message
        super().__init__(reason=None, message=message)

    def __str__(self):
        return self.message
