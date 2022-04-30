import json

import rospy
import uvicorn
from fastapi import FastAPI, Request, status
from starlette.responses import Response
from std_msgs.msg import String

API_HOST = "127.0.0.1"
API_PORT = 5000

app = FastAPI()
pub = rospy.Publisher("intent", String, queue_size=10)
rospy.init_node("intent_api_service")


@app.post("/intent", status_code=status.HTTP_204_NO_CONTENT, response_class=Response)
async def receive_intent(request: Request):
    data = await request.json()
    publish_intent(data)


def publish_intent(intent):
    json_string = json.dumps(intent)
    pub.publish(json_string)


if __name__ == "__main__":
    uvicorn.run("api:app", host=API_HOST, port=API_PORT, log_level="info")

# curl -X POST http://localhost:5000/intent -H "Content-Type: application/json" -d '{"message":"Hello"}'
