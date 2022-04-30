import threading
from typing import List

from src.api.action_api_client import ActionApiClient
from src.core.action_organizer import ActionOrganizer
from src.core.action_queue import ActionQueue
from src.core.model.action.atomic.pose_action import PoseAction
from src.core.model.action.group.default_action_group import DefaultActionGroup
from src.util.config import ConfigProvider
from src.util.ros_node import start_ros_node


class Node:
    def start(self):
        start_ros_node()
        ConfigProvider.init_config()
        action_queue = ActionQueue()

        action_queue.push(
            DefaultActionGroup(
                actions=[
                    PoseAction(start_ms=0, end_ms=2000),
                    PoseAction(start_ms=4000, end_ms=4500),
                    PoseAction(start_ms=2500, end_ms=2900),
                ]
            )
        )

        threads = self.__start_threads(action_queue)

        for thread in threads:
            thread.join()

    @staticmethod
    def __start_api_client(queue: ActionQueue):
        action_api_client = ActionApiClient(action_queue=queue)
        action_api_client.start()

    @staticmethod
    def __start_action_organizer(queue: ActionQueue):
        action_organizer = ActionOrganizer(action_queue=queue)
        action_organizer.start()

    def __start_threads(self, queue: ActionQueue) -> List[threading.Thread]:
        api_service_thread = threading.Thread(target=self.__start_api_client, args=(queue,))
        action_organizer_thread = threading.Thread(target=self.__start_action_organizer, args=(queue,))

        api_service_thread.start()
        action_organizer_thread.start()

        return [api_service_thread, action_organizer_thread]


if __name__ == "__main__":
    Node().start()
