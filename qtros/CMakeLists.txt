
cmake_minimum_required(VERSION 2.8.0)
project(qtros)

add_compile_options(-std=c++11)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED
    COMPONENTS
    qt_build
    roscpp
    rospy
    std_msgs
    cv_bridge
    sensor_msgs
    image_transport
    genmsg
    message_filters
    find_object_2d
    tf
    message_generation
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


include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

include_directories(SYSTEM ${THIS_PACKAGE_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})

generate_messages(DEPENDENCIES std_msgs sensor_msgs)


catkin_package(
  CATKIN_DEPENDS
  cv_bridge
  roscpp
  rospy
  sensor_msgs
  std_msgs
  image_transport
  message_filters
  find_object_2d
  tf
)



catkin_package(
  LIBRARIES interactivity_utils
  INCLUDE_DIRS include
  LIBRARIES moveit_mr5_config
  CATKIN_DEPENDS roscpp moveit_core moveit_visual_tools moveit_ros_planning_interface interactive_markers
  DEPENDS system_lib EIGEN3
)

rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

##############################################################################
# Sections${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/qtros/*.hpp)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

SET(QT_USE_QTNETWORK true)
##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################

add_executable(qtros ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(qtros ${QT_LIBRARIES} ${catkin_LIBRARIES}  ${OpenCV_LIBS} ${QT_QTNETWORK_LIBRARY} ${Boost_LIBRARIES})
install(TARGETS qtros RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


