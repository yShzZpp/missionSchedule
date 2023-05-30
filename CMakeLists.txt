cmake_minimum_required(VERSION 3.0.2)
project(mission_schedule VERSION 1.2.0)
set(PATCH_VERSION_INFO "y")

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-DPROJECT_NAME="${PROJECT_NAME}")
add_compile_options(-DMAIN_VERSION_INFO="${PROJECT_VERSION}")
add_compile_options(-DPATCH_VERSION_INFO="${PATCH_VERSION_INFO}")
execute_process(
    COMMAND git rev-parse --abbrev-ref HEAD
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_BRANCH
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
execute_process(
    COMMAND git config user.name
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_USER_NAME
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

execute_process(
    COMMAND git config user.email
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_USER_EMAIL
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

string(TIMESTAMP COMPILED_TIME "%Y-%m-%d %H:%M:%S")
add_compile_options(-DCOMPILED_TIME="${COMPILED_TIME}")
add_compile_options(-DGIT_BRANCH="${GIT_BRANCH}")
add_compile_options(-DGIT_USER_NAME="${GIT_USER_NAME}")
add_compile_options(-DGIT_USER_EMAIL="${GIT_USER_EMAIL}")

find_package (catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  tf
  nav_msgs
  cti_msgs
  std_msgs
  geometry_msgs
  common
  json
  spdlog
  behaviortree_cpp_v3
  save_crash_log
  message_generation
  road_control
)

file(GLOB_RECURSE project_cxx_sources "${PROJECT_SOURCE_DIR}/src/*.cpp")

find_package(Boost REQUIRED system)
find_package(Protobuf REQUIRED)
find_package(PNG REQUIRED)

add_message_files(
  FILES
  batteryMsg.msg
  rebootTimeMsg.msg
  coordinatesMsg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES collvoid_local_schedule
  CATKIN_DEPENDS message_runtime geometry_msgs std_msgs common json spdlog save_crash_log behaviortree_cpp_v3
#  DEPENDS system_lib
)

include_directories(
 include
 ${PROTOBUF_INCLUDE_DIR}
 ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node ${project_cxx_sources})

add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  ${PNG_LIBRARY}
  yaml-cpp
  )

#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
