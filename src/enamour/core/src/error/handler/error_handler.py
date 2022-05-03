import abc


class ErrorHandler(BaseException):
    @abc.abstractmethod
    def handle(self, error: BaseException, **kwargs):
        raise NotImplementedError
