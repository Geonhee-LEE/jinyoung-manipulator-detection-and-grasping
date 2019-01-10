cmake_minimum_required(VERSION 2.8.3)
project(camera_node)

add_compile_options(-std=c++11)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED COMPONENTS
  qt_build
  cv_bridge
  image_transport
  roscpp
  std_msgs
  rospy
  sensor_msgs
  image_transport
  genmsg
  message_filters
  tf
  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  geometric_shapes
  pcl_ros
  pcl_conversions
  rosbag
)


find_package(OpenCV REQUIRED)

find_package(Boost REQUIRED system filesystem date_time thread)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${QT_INCLUDE_DIRS})

include_directories(SYSTEM ${THIS_PACKAGE_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})


catkin_package(
  CATKIN_DEPENDS cv_bridge roscpp rospy sensor_msgs std_msgs image_transport message_filters tf qt_build
)

catkin_package(
  LIBRARIES interactivity_utils
  INCLUDE_DIRS include
  LIBRARIES moveit_mr5_config
  CATKIN_DEPENDS roscpp moveit_core moveit_visual_tools moveit_ros_planning_interface interactive_markers
  DEPENDS system_lib EIGEN3
)
rosbuild_prepare_qt4(QtCore QtGui)



QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})



add_executable(camera_node src/camera_node.cpp ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(camera_node  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${QT_LIBRARIES}  ${Boost_LIBRARIES})




