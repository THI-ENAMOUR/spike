#!/usr/bin/env python3

import json
from json import JSONDecodeError

import rospy
import uvicorn
from fastapi import FastAPI, Request, status
from starlette.responses import Response
from std_msgs.msg import String

from api_error import ApiError

API_HOST = "127.0.0.1"
API_PORT = 5000

app = FastAPI()

pub = rospy.Publisher("intent", String, queue_size=10)
rospy.init_node("intent_api_service")


@app.post("/intent")
async def receive_intent(request: Request):
    try:
        data = await request.json()
        publish_intent(data)
    except JSONDecodeError:
        return ApiError(title="Invalid request payload format",
                        message="The payload is no valid json",
                        http_status=status.HTTP_400_BAD_REQUEST).to_response()

    return Response(status_code=status.HTTP_204_NO_CONTENT)


def publish_intent(intent):
    json_string = json.dumps(intent)
    pub.publish(json_string)


if __name__ == "__main__":
    uvicorn.run(app, host=API_HOST, port=API_PORT, log_level="info")
