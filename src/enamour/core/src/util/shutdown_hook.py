import signal


class ShutdownHook(object):
    is_running = True

    @staticmethod
    def on_shutdown(*args):
        """Called when the application receives a shutdown signal"""
        pass


# Do not remove *args from function signature. It is required.
def __call_on_shutdown(*args):
    ShutdownHook.is_running = False
    ShutdownHook.on_shutdown()


# Hook into the systems terminal signal and call the shutdown callback function.
# This allows the application to clean up resources.
signal.signal(signal.SIGINT, __call_on_shutdown)
signal.signal(signal.SIGTERM, __call_on_shutdown)
