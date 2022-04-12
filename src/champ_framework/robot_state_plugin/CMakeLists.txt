cmake_minimum_required(VERSION 2.8.12)
project(robot_state_plugin)

add_compile_options(-std=c++11)

# definition needed for boost/math/constants/constants.hpp included by Ogre to compile
add_definitions(-DBOOST_MATH_DISABLE_FLOAT128)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED thread date_time system filesystem)

find_package(PkgConfig REQUIRED)

# Ogre
pkg_check_modules(OGRE OGRE)
link_directories( ${OGRE_LIBRARY_DIRS} )

find_package(catkin REQUIRED COMPONENTS
  class_loader
  moveit_ros_planning_interface
  pluginlib
  roscpp
  rviz
)

# Qt Stuff
if(rviz_QT_VERSION VERSION_LESS "5")
	find_package(Qt4 ${rviz_QT_VERSION} REQUIRED QtCore QtGui)
	include(${QT_USE_FILE})
  macro(qt_wrap_ui)
    qt4_wrap_ui(${ARGN})
  endmacro()
else()
	find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
	set(QT_LIBRARIES Qt5::Widgets)
  macro(qt_wrap_ui)
    qt5_wrap_ui(${ARGN})
  endmacro()
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
  LIBRARIES
    moveit_rviz_plugin_render_tools
    moveit_robot_state_rviz_plugin_core
    moveit_planning_scene_rviz_plugin_core
  INCLUDE_DIRS
    rviz_plugin_render_tools/include
    robot_state_rviz_plugin/include
  CATKIN_DEPENDS
    moveit_ros_planning_interface
  DEPENDS
)

include_directories(rviz_plugin_render_tools/include
                    robot_state_rviz_plugin/include
                    ${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS})

include_directories(SYSTEM
                    ${QT_INCLUDE_DIR}
                    ${OGRE_INCLUDE_DIRS})

add_subdirectory(rviz_plugin_render_tools)
add_subdirectory(robot_state_rviz_plugin)

install(FILES
  robot_state_rviz_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY icons DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
