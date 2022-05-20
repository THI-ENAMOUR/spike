import json
import uuid
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


def to_json(data, **kwds):
    return json.dumps(data, default=to_serializable_dict, **kwds)


def to_serializable_dict(data):
    if is_primitive(data) or data is None:
        return data
    elif isinstance(data, uuid.UUID):
        return str(data)
    if hasattr(data, "__dict__"):
        deserializable_dict = data.__dict__
        for key, value in deserializable_dict.iteritems():
            if isinstance(value, list):
                new_value = []
                for item in value:
                    new_value.append(to_serializable_dict(item))
                deserializable_dict[key] = new_value
            elif isinstance(value, uuid.UUID):
                deserializable_dict[key] = str(value)
        return deserializable_dict
    else:
        return str(data)


def is_primitive(data):
    return isinstance(data, (int, float, bool, str))
