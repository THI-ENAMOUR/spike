# ENAMOUR Core

The core of the enamour application. It receives intents and publishes them on a ros topic.
Further it receives actions and adds them to the action queue. Every X milliseconds the current action is fetched
from the queue and executed. The controllers are responsible for actually applying the actions to the
external systems like the robot, display or speaker.

## Run the application

Build the project with catkin_make or the make.sh script and then execute:

- `roslaunch enamour_core bringup.launch`