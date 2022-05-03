class ApplicationError(RuntimeError):
    """Generic error occurring inside the application."""

    def __init__(self, is_critical: bool = True):
        self.is_critical = is_critical
