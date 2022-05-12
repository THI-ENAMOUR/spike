class ApplicationError(RuntimeError):
    """Generic error occurring inside the application."""

    def __init__(self, message, is_critical=True):
        self.message = message
        self.is_critical = is_critical
        """Marks the error as critical (default=True)."""

    def __str__(self):
        return self.message
