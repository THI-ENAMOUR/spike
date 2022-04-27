from fastapi import FastAPI, Request, status
from rospy_message_converter import json_message_converter
import uvicorn
from std_msgs.msg import String
import rospy

API_HOST = "127.0.0.1"
API_PORT = 5000

app = FastAPI()
pub = rospy.Publisher('chatter', String, queue_size=10)
rospy.init_node('talker', anonymous=True)


@app.post("/set-ki-intent", status_code=status.HTTP_204_NO_CONTENT)
async def update(request: Request):
    data = await request.json()
    publish(data)
    return


def publish(message):
    data = json_message_converter.convert_json_to_ros_message('std_msgs/String', message)
    pub.publish(data)
    return None


if __name__ == "__main__":
    uvicorn.run("api:app", host=API_HOST, port=API_PORT, log_level="info")



# curl -X POST http://localhost:5000/set-ki-intent -H "Content-Type: application/json" -d '{"message":"Hello"}'
