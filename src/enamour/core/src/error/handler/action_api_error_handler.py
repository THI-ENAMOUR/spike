from error.handler.error_handler import ErrorHandler


class ActionApiErrorHandler(ErrorHandler):
    def handle(self, error: BaseException, **kwargs):
        print(f"Error occurred in action api service with message: {error}")
