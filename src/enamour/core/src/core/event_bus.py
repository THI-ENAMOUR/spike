import abc
import string
from typing import TypeVar, Any, Callable

import rospy

T = TypeVar("T")


class Publisher:
    def publish(self, *args, **kwds):
        raise NotImplementedError


# Abstraction of an event bus in order to be able to test with a simulated bus and switch between implementations
class EventBus(metaclass=abc.ABCMeta):
    """Abstract class for an event bus"""

    @abc.abstractmethod
    def init_node(self, name: string):
        raise NotImplementedError

    @abc.abstractmethod
    def get_parameter(self, name: string, default: string):
        raise NotImplementedError

    @abc.abstractmethod
    def subscribe(self, topic: string, data_type: T, callback: Callable[[T], Any]):
        raise NotImplementedError

    @abc.abstractmethod
    def publish(self, topic: string, data_type: T, queue_size: int = 10) -> Publisher:
        raise NotImplementedError

    @abc.abstractmethod
    def is_running(self):
        return False

    @abc.abstractmethod
    def spin(self):
        pass


class RosEventBus(EventBus):
    def init_node(self, name: string):
        rospy.init_node(name)

    def get_parameter(self, name: string, default: string):
        return rospy.get_param(name, default)

    def subscribe(self, topic: string, data_class: T, callback: Callable[[T], Any]):
        rospy.Subscriber(name=topic, data_class=data_class, callback=callback)

    def publish(self, topic: string, data_class: T, queue_size: int = 10) -> Publisher:
        return rospy.Publisher(name=topic, data_class=data_class, queue_size=queue_size)

    def is_running(self):
        return not rospy.is_shutdown()

    def spin(self):
        rospy.spin()


event_bus: EventBus = RosEventBus()
