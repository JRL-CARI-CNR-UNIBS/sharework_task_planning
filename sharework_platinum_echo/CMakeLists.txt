cmake_minimum_required(VERSION 3.0.2)
project(sharework_platinum_echo)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  subscription_notifier
  task_planner_interface_msgs
)

catkin_package(
  INCLUDE_DIRS #include
  LIBRARIES
  CATKIN_DEPENDS roscpp std_msgs subscription_notifier task_planner_interface_msgs
  DEPENDS
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(sharework_platinum_echo_node src/sharework_platinum_echo_node.cpp)
add_dependencies(sharework_platinum_echo_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sharework_platinum_echo_node
   ${catkin_LIBRARIES}
)

FILE(GLOB_RECURSE YamlFiles "config/*.yaml")
add_custom_target(yaml SOURCES ${YamlFiles})

FILE(GLOB_RECURSE LaunchFiles "launch/*.launch")
add_custom_target(launch SOURCES ${LaunchFiles})

