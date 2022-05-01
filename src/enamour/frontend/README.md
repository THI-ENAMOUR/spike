# Frontend
This ROS package allows to display the facial expressions and play sounds of Spike in a web browser. The web browser establishes an SSE connection with the web server. Updates of emotion are automatically forwarded to the client through this connection. An additional ROS node converts the received ROS messages into an HTTP POST command and initiates the update process. Sounds or images can be added later for new emotions. 

## Install Dependencies
Before you start the package, make sure that all dependencies are installed
```
pip install -r requirements.txt
```

## Build your workspace:
```
cd <your_ws>
catkin_make
source <your_ws>/devel/setup.bash
```

## Start the package
The next command starts the web server and the ROS node.
- `roslaunch enamour_frontend bringup.launch`
- OR: `rosrun enamour_frontend app.py`

## Test
Visit the website. The server listens on port 5000 by default. Make sure that your browser allows automatic playback of video and audio, otherwise the sound will not play. <br>
Send an update (e.g. with rostopic):
```
rostopic pub /cmd_facial_expression std_msgs/String "data: '{\"expression\": \"default\"}'"
```
Now the website should be updated.

## Add new images and sounds
Save your new image or audio files in the `server/static/display` or `server/static/audio` folder. <br>
Map your added audio and image file to an expression in the `server/face-expression.json`:
```
{
    "angry": 
    {
        "display": "angry.gif",
        "audio": "angry.mp3"
    }
}
```

