from error.validation_error import ValidationError


class DeserializationError(ValidationError):
    """Thrown if the deserialization of a resource failed."""

    def __init__(self, message: str):
        super().__init__(reason=None, message=message)
