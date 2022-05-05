from typing import Optional

from error.application_error import ApplicationError


class ValidationError(ApplicationError):
    """Thrown if a validation failed."""

    def __init__(self, reason: Optional[str], is_critical: bool = True, message: str = None):
        message = message if message is not None else f"Validation failed: {reason}"
        super().__init__(message=message, is_critical=is_critical)
