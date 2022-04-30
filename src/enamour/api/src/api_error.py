import json
import string

from starlette.responses import Response


class ApiError:
    def __init__(self, title: string, message: string, http_status: int = 400):
        self.title = title
        self.message = message
        self.status = http_status

    def to_response(self) -> Response:
        return Response(content=json.dumps(self.__dict__, indent=4), status_code=self.status)