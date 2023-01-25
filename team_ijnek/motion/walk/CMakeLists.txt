cmake_minimum_required(VERSION 3.5)
project(walk)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(motion_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nao_sensor_msgs REQUIRED)
find_package(nao_command_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(biped_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(walk_msg REQUIRED)
find_package(walk_sensor_msg REQUIRED)
find_package(message_filters REQUIRED)
find_package(walk_utils REQUIRED)

ament_export_include_directories(include)

# Build walk

add_library(sensorvalues src/Sensors.cpp)
ament_target_dependencies(sensorvalues
  std_msgs
  nao_sensor_msgs
  nao_command_msgs)

add_library(actioncommand src/ActionCommand.cpp)
ament_target_dependencies(actioncommand
  walk_msg)

add_library(bodymodel src/BodyModel.cpp)
ament_target_dependencies(bodymodel
  nao_sensor_msgs
  nao_command_msgs
  std_msgs)

add_library(bodyv6 src/bodyV6.cpp)
ament_target_dependencies(bodyv6)

add_library(mathfunctions src/maths_functions.cpp)
ament_target_dependencies(mathfunctions)

add_library(walkcycle src/WalkCycle.cpp)
ament_target_dependencies(walkcycle)

add_library(walk src/walk.cpp)
ament_target_dependencies(walk
  motion_interfaces
  std_msgs
  nao_sensor_msgs
  nao_command_msgs
  tf2_geometry_msgs
  biped_interfaces
  walk_msg
  message_filters
  walk_sensor_msg
  walk_utils)

# Build walk_node
add_library(walk_node src/walk_node.cpp)
target_link_libraries(walk_node walk sensorvalues)
ament_target_dependencies(walk_node
  rclcpp)

# Install
install(TARGETS
  walk_node
  DESTINATION lib/${PROJECT_NAME})

install(
  TARGETS walk
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)


ament_package()
