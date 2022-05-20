from uuid import UUID

from error.deserialization_error import DeserializationError


def get(data, key, expected_type=None):
    try:
        value = data[key]
        if value is None:
            raise DeserializationError(key + " not found in json")
        if expected_type is not None and type(value) is not expected_type:
            raise DeserializationError(key + " is not of the correct type")

        return value
    except KeyError:
        raise DeserializationError(key + " not found in json")


def get_default(data, key, default):
    try:
        value = get(data, key)
        return value if not None else default
    except DeserializationError:
        return default


def to_UUID(key):
    try:
        return UUID(key, version=4)
    except ValueError:
        raise DeserializationError(key + " is not an valid uuid version 4")
