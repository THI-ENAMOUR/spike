import abc


class ErrorHandler(BaseException):
    """Handles the error management of a component"""

    @abc.abstractmethod
    def handle(self, error, **kwargs):
        raise NotImplementedError
