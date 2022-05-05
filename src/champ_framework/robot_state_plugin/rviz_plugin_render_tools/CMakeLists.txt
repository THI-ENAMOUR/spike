set(MOVEIT_LIB_NAME moveit_rviz_plugin_render_tools)

set(HEADERS
  include/moveit/rviz_plugin_render_tools/planning_link_updater.h
  include/moveit/rviz_plugin_render_tools/robot_state_visualization.h
)

include_directories(${CMAKE_CURRENT_BINARY_DIR}
  ${PROJECT_BUILD_DIR}/rviz_plugin_render_tools
)

add_library(${MOVEIT_LIB_NAME}
  src/robot_state_visualization.cpp
  src/planning_link_updater.cpp
  ${HEADERS}
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})

target_link_libraries(${MOVEIT_LIB_NAME}
  ${catkin_LIBRARIES}
  ${OGRE_LIBRARIES}
  ${QT_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

install(TARGETS ${MOVEIT_LIB_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
