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

        # Ignore unknown type 'unicode' in code linter with the comment 'noqa: F821'
        value = value if not isinstance(value, unicode) else str(value)  # noqa: F821
        return value
    except KeyError:
        raise DeserializationError(key + " not found in json")


def get_default(data, key, default):
    try:
        value = get(data, key)
        value = value if not isinstance(value, unicode) else str(value)  # noqa: F821

        return value if not None else default
    except DeserializationError:
        return default


def to_UUID(key):
    try:
        return UUID(key, version=4)
    except ValueError:
        raise DeserializationError(key + " is not an valid uuid version 4")


def to_json(data):
    return json.dumps(data, default=to_serializable_dict)


def to_serializable_dict(data):
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
    elif data is None:
        # Just return 'None' so json.dumps changes it to 'null'
        return None
    else:
        return str(data)
