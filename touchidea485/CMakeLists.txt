################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(touchidea485)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES touchidea485
)

################################################################################
# Build
################################################################################
include_directories(
  include/touchidea485
  ${catkin_INCLUDE_DIRS}
)

add_library(touchidea485
  src/touchidea485/packet_handler.cpp
  src/touchidea485/protocol1_packet_handler.cpp
  src/touchidea485/protocol2_packet_handler.cpp
  src/touchidea485/group_sync_read.cpp
  src/touchidea485/group_sync_write.cpp
  src/touchidea485/group_bulk_read.cpp
  src/touchidea485/group_bulk_write.cpp
  src/touchidea485/port_handler.cpp
  src/touchidea485/port_handler_linux.cpp
)
add_dependencies(touchidea485 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(touchidea485 ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

################################################################################
# Test
################################################################################
