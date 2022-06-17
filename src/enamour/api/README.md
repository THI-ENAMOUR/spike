# API-Service for communication between teams

This node provides an API-Service for receiving intent data and publishing
them on a ROS topic.

### Running this node in ROS:
- `rosrun enamour_api __main__.py`
- OR: `roslaunch enamour_api bringup.launch`

### API-Information:
- Intent Topic Name: __\intent__
- Port: __4000__
- Path: __POST /intent__
- Body: Intent as json payload
- Success Response: 204
- Error Response: 400
- Testing Command:
  - `curl -X POST http://localhost:4000/intent -H "Content-Type: application/json" -d '{"message":"Hello"}'`