cmake_minimum_required(VERSION 2.8.3)
project(xml_class_parser)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  # ROS dependencies
  rospy
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

# Testing
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  
  foreach(T
      test/test-parsable.launch)
     add_rostest(${T})
  endforeach()
endif()