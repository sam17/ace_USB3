
cmake_minimum_required(VERSION 2.8.3)
project(ace_pleora_wrapper)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(Ebus REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  message_generation
  roscpp
  sensor_msgs
  tf
  image_transport
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES compute_cdist
  CATKIN_DEPENDS cv_bridge message_generation roscpp sensor_msgs tf
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${Ebus_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

#add_library(${PROJECT_NAME}
#  src/ace_usb3.cpp
#  src/ace_pleora_wrapper_node.cpp
#  )


add_executable(ros_pleora
  src/ace_pleora_wrapper.cpp
  src/ace_usb3.cpp)

target_link_libraries(ros_pleora
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${Ebus_LIBRARIES}
    )

#target_link_libraries(${PROJECT_NAME}_node
# ${PROJECT_NAME}
# )
