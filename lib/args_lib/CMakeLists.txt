cmake_minimum_required(VERSION 2.8.3)
project(args_lib)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  # ROS c++ dependencies
  roscpp
  rospy

  # Messages base dependencies
  message_generation
  std_msgs
)

catkin_python_setup()

# Messages
add_message_files(DIRECTORY msg
  FILES
  Argument.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy message_runtime std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/Argumentable.cpp
)

# Add cmake target dependencies of the library
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Testing
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  foreach(T #insert unit test here
    )
    add_rostest(${T})
  endforeach()
endif()