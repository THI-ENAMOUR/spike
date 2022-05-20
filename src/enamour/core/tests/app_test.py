import unittest


class AppTestCases:
    def __init__(self):
        pass

    class AppTest(unittest.TestCase):
        def assertThrown(self, func, error_type=None, on_error_func=None):
            try:
                func()
            except BaseException as error:
                if error_type is not None:
                    self.assertTrue(isinstance(error, error_type))
                if on_error_func is not None:
                    on_error_func(error)
