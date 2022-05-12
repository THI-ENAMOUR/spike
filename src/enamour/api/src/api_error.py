import json


class ApiError:
    def __init__(self, title, message, http_status=400):
        self.title = title
        self.message = message
        self.status = http_status

    def to_json(self):
        return json.dumps(self.__dict__)
