import datetime
import threading
import unittest
from time import sleep

from tests.app_test import AppTestCases
from util.synchronized import synchronized


@synchronized
class SynchronizedClass(object):
    def func1(self, func):
        func()

    def func2(self, func):
        func()


@synchronized
def synchronized_func(func):
    return func()


class TestJsonMapper(AppTestCases.AppTest):
    def test_class_annotation(self):
        synch = SynchronizedClass()
        thread1 = threading.Thread(target=lambda: synch.func1(func=lambda: sleep(0.5)))
        thread2 = threading.Thread(target=lambda: synch.func2(func=lambda: "Do nothing"))

        start_time = datetime.datetime.now()
        thread1.start()
        thread2.start()
        thread2.join()
        end_time = datetime.datetime.now()
        time_diff = end_time - start_time
        execution_time = time_diff.total_seconds() * 1000
        self.assertTrue(execution_time > 500)
        thread1.join()

    def test_func_annotation(self):
        thread1 = threading.Thread(target=lambda: synchronized_func(func=lambda: sleep(0.5)))
        thread2 = threading.Thread(target=lambda: synchronized_func(func=lambda: "Do nothing"))

        start_time = datetime.datetime.now()
        thread1.start()
        thread2.start()
        thread2.join()
        end_time = datetime.datetime.now()
        time_diff = end_time - start_time
        execution_time = time_diff.total_seconds() * 1000
        self.assertTrue(execution_time > 500)
        thread1.join()

    def test_no_deadlock_if_same_thread(self):
        thread = threading.Thread(target=lambda: synchronized_func(lambda: synchronized_func(lambda: "")))
        thread.start()
        thread.join(timeout=0.5)
        self.assertFalse(thread.isAlive())


if __name__ == "__main__":
    unittest.main()
