cmake_minimum_required(VERSION 2.8.3)
# to make CMake shut up about some fckin policies
cmake_policy(SET CMP0048 NEW)
project(label_extraction VERSION 666)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  image_geometry
  message_generation
  tf2
  tf2_geometry_msgs
  dynamic_reconfigure
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options("-Wall")
add_compile_options("-Wextra")
add_compile_options("-g")
add_compile_options("-Og")

add_message_files(DIRECTORY msg FILES
  Label.msg
  Labels.msg
)

generate_messages(DEPENDENCIES
  std_msgs
  sensor_msgs
)

generate_dynamic_reconfigure_options(
  config/Visualization.cfg
)

catkin_package(
  CATKIN_DEPENDS message_runtime image_transport
)
