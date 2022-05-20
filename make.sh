source /opt/ros/melodic/setup.bash

catkin_make

if [[ -z "${PROJECT_SCRIPTS_DIR}" ]]; then
  ENVIRONMENT_SCRIPT="scripts/setup_environment.sh"
else
  ENVIRONMENT_SCRIPT="${PROJECT_SCRIPTS_DIR}/setup_environment.sh"
fi


source $ENVIRONMENT_SCRIPT
