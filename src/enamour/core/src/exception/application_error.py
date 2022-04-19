class ApplicationError(RuntimeError):
    """Generic error occurring inside the application."""

    def __init__(self, *args, **kwargs):
        super(ApplicationError, self).__init__(args, kwargs)
