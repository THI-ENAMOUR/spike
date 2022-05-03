class ApplicationError(RuntimeError):
    """Generic error occurring inside the application."""

    def __init__(self, message: str, is_critical: bool = True):
        self.message = message
        self.is_critical = is_critical
        """Marks the error as critical (default=True)."""

    def __str__(self):
        return self.message
