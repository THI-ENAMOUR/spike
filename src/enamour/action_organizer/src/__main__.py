import threading
from typing import List

from src.api.action_api_client import ActionApiClient
from src.api.action_factory import ActionFactory
from src.core.action_organizer import ActionOrganizer
from src.core.action_queue import ActionQueue
from src.core.model.action.high_level.sit_action import SitAction
from src.util.config import ConfigProvider
from src.util.ros_node import start_ros_node


# TODO: Add logging library that works with ros AND console. Add tests. Check use cases if this architecture can handel
# TODO: all of them. Rename classes to better names. Add exception management.


class Node:
    def start(self):
        start_ros_node()
        ConfigProvider.init_config()
        action_queue = ActionQueue()

        action_queue.push(SitAction())

        threads = self.__start_threads(action_queue)

        for thread in threads:
            thread.join()

    @staticmethod
    def __start_api_client(queue: ActionQueue):
        action_factory = ActionFactory()
        action_api_client = ActionApiClient(action_queue=queue, action_factory=action_factory)
        # action_api_client.receive_action_request("{}")

    @staticmethod
    def __start_action_organizer(queue: ActionQueue):
        action_organizer = ActionOrganizer(action_queue=queue)
        action_organizer.run()

    def __start_threads(self, queue: ActionQueue) -> List[threading.Thread]:
        api_service_thread = threading.Thread(target=self.__start_api_client, args=(queue,))
        action_organizer_thread = threading.Thread(target=self.__start_action_organizer, args=(queue,))

        api_service_thread.start()
        action_organizer_thread.start()

        return [api_service_thread, action_organizer_thread]


if __name__ == "__main__":
    Node().start()
