cmake_minimum_required(VERSION 3.0)
project(object_tracking_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -g -Wall)

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  cv_bridge
  std_msgs
  roscpp
  message_generation
  image_transport
  #pixelink_ros
)

#set(PXL_LIB_PATH $ENV{PIXELINK_SDK_LIB})
#set(PXL_INC $ENV{PIXELINK_SDK_INC})

#find_library(PXL_LIB PATHS ${PXL_LIB_PATH} NAMES PxLApi)

  
generate_messages(DEPENDENCIES sensor_msgs
)

catkin_package(CATKIN_DEPENDS
  std_msgs
  sensor_msgs
  roscpp
  message_runtime
)


#########
# BUILD #
#########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/src
  ${OpenCV_INCLUDE_DIRS}
)
#add_definitions(-DPIXELINK_LINUX)

add_executable(object_tracking_node
               src/object_tracking_ros.cpp
               src/tracker.cpp)

set(EXEC_LIST object_tracking_node)
foreach(EXEC_FILE ${EXEC_LIST})
  target_link_libraries(${EXEC_FILE} ${OpenCV_LIBS} ${catkin_LIBRARIES})
  add_dependencies(${EXEC_FILE} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
  install(TARGETS ${EXEC_FILE}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach(EXEC_FILE ${EXEC_LIST})



