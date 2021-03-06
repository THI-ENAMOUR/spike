import threading

from api.action_api_client import ActionApiClient
from core.action_organizer import ActionOrganizer
from core.action_queue import ActionQueue
from util.logger import Logger
from util.shutdown_hook import ShutdownHook


class Application(object):
    __logger = Logger(__name__)

    def __init__(self, action_queue=ActionQueue()):
        self.action_queue = action_queue
        self.action_api_client = None
        self.action_organizer = None

        # Execute shutdown action if the application receives an external shutdown signal
        ShutdownHook.on_shutdown = self.shutdown

    def start(self):
        self.__logger.info("Start core node")
        threads = self.__start_threads(self.action_queue)

        for thread in threads:
            # Do not block the main thread, else the shutdown hook will not be triggered
            while ShutdownHook.is_running and thread.isAlive():
                thread.join(0.5)

    def shutdown(self):
        self.__logger.info("Shutting down node")
        if self.action_api_client is not None:
            self.action_api_client.running = False
        if self.action_organizer is not None:
            self.action_organizer.running = False

    def __start_threads(self, queue):
        """Start a thread for each dependant test to increase performance"""

        api_service_thread = threading.Thread(target=self.__start_api_client, args=(queue,))
        action_organizer_thread = threading.Thread(target=self.__start_action_organizer, args=(queue,))

        api_service_thread.start()
        action_organizer_thread.start()

        return [api_service_thread, action_organizer_thread]

    def __start_api_client(self, queue):
        self.action_api_client = ActionApiClient(action_queue=queue)
        self.action_api_client.start()

    def __start_action_organizer(self, queue):
        self.action_organizer = ActionOrganizer(action_queue=queue)
        self.action_organizer.start()
