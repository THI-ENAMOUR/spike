import datetime
import unittest
import uuid

from error.deserialization_error import DeserializationError
from tests.app_test import AppTestCases
from util.json_mapper import to_UUID, to_json, get_default, get


class JsonData(object):
    def __init__(self):
        self.id = to_UUID("22fd4cde-2c15-49ad-a840-f83eda86a8fa")
        self.bool_true = True
        self.bool_false = False
        self.null = None
        self.float = 28.125129
        self.list = [None, ChildObject(), to_UUID("2d3226e1-2574-4dab-97dc-f44bbe16d55b"), 5, "Test"]


class ChildObject(object):
    def __init__(self):
        self.id = to_UUID("38f11c4f-738b-4299-b424-45d35d9447db")
        self.date = datetime.datetime.strptime("2022-05-20 16:28:13", "%Y-%m-%d %H:%M:%S")


expected_json = """{
    "bool_false": false,
    "bool_true": true,
    "float": 28.125129,
    "id": "22fd4cde-2c15-49ad-a840-f83eda86a8fa",
    "list": [
        null,
        {
            "date": "2022-05-20 16:28:13",
            "id": "38f11c4f-738b-4299-b424-45d35d9447db"
        },
        "2d3226e1-2574-4dab-97dc-f44bbe16d55b",
        5,
        "Test"
    ],
    "null": null
}"""


class TestJsonMapper(AppTestCases.AppTest):
    def test_to_uuid(self):
        expected = "2d3226e1-2574-4dab-97dc-f44bbe16d55b"
        actual = to_UUID("2d3226e1-2574-4dab-97dc-f44bbe16d55b")
        self.assertTrue(isinstance(actual, uuid.UUID))
        self.assertEqual(expected, str(actual))

    def test_to_json(self):
        json_data = JsonData()
        actual = to_json(json_data, indent=4, sort_keys=True).replace(", ", ",")  # No trailing spaces
        self.assertEqual(expected_json, actual)

    def test_get_default(self):
        data = {"key": "expected_data"}
        actual_none = get_default(data, "key", None)
        expected_none = data["key"]
        actual_default = get_default(data, "invalid", 4.2)
        expected_default = 4.2
        self.assertEqual(expected_none, actual_none)
        self.assertEqual(expected_default, actual_default)

    def test_get(self):
        data = {"key": "expected_data"}
        actual = get(data, "key")
        expected = data["key"]
        self.assertEqual(expected, actual)

        self.assertRaises(DeserializationError, get, data, "invalid")
        self.assertRaises(DeserializationError, get, data, "key", float)


if __name__ == "__main__":
    unittest.main()
