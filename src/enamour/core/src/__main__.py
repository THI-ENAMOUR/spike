#!/usr/bin/env python3

import threading
from typing import List, Optional

from api.action_api_client import ActionApiClient
from core.action_organizer import ActionOrganizer
from core.action_queue import ActionQueue
from core.event_bus import event_bus
from core.model.action.atomic.pose_action import PoseAction
from core.model.action.group.default_action_group import DefaultActionGroup
from util.config import Config
from util.shutdown_hook import ShutdownHook


class Node:
    def __init__(self, action_queue: ActionQueue = ActionQueue()):
        event_bus.init_node(event_bus.get_parameter("core_node_name", "enamour_core"))
        Config.init_config()
        self.action_queue = action_queue
        self.action_api_client: Optional[ActionApiClient] = None
        self.action_organizer: Optional[ActionOrganizer] = None
        ShutdownHook.on_shutdown = self.shutdown

    def start(self):
        threads = self.__start_threads(self.action_queue)

        for thread in threads:
            # Block until every thread is done
            thread.join()

    def shutdown(self):
        print("Shutting down node")
        if self.action_api_client is not None:
            self.action_api_client.running = False
        if self.action_organizer is not None:
            self.action_organizer.running = False

    def __start_threads(self, queue: ActionQueue) -> List[threading.Thread]:
        api_service_thread = threading.Thread(target=self.__start_api_client, args=(queue,))
        action_organizer_thread = threading.Thread(target=self.__start_action_organizer, args=(queue,))

        api_service_thread.start()
        action_organizer_thread.start()

        return [api_service_thread, action_organizer_thread]

    def __start_api_client(self, queue: ActionQueue):
        self.action_api_client = ActionApiClient(action_queue=queue)
        self.action_api_client.start()

    def __start_action_organizer(self, queue: ActionQueue):
        self.action_organizer = ActionOrganizer(action_queue=queue)
        self.action_organizer.start()


if __name__ == "__main__":
    __queue = ActionQueue()
    node = Node(action_queue=__queue)

    __queue.push(
        DefaultActionGroup(
            actions=[
                PoseAction(start_ms=0, end_ms=2000),
                DefaultActionGroup(
                    actions=[
                        PoseAction(start_ms=0, end_ms=2000),
                        DefaultActionGroup(
                            actions=[
                                PoseAction(start_ms=4000, end_ms=4500),
                                PoseAction(start_ms=2500, end_ms=2900),
                            ]
                        ),
                        PoseAction(start_ms=4000, end_ms=4500),
                        PoseAction(start_ms=2500, end_ms=2900),
                    ]
                ),
                PoseAction(start_ms=2001, end_ms=5000),
            ]
        )
    )
    node.start()
