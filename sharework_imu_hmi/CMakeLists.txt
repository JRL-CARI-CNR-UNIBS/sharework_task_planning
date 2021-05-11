cmake_minimum_required(VERSION 3.0.2)
project(sharework_imu_hmi)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  ros_myo
  roscpp
  rospy
  std_msgs
  task_planner_interface_msgs
  subscription_notifier
)

catkin_package(
  INCLUDE_DIRS #include
  LIBRARIES
  CATKIN_DEPENDS ros_myo roscpp rospy std_msgs task_planner_interface_msgs subscription_notifier
  DEPENDS
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(sharework_imu_hmi_node src/sharework_imu_hmi_node.cpp)
add_dependencies(sharework_imu_hmi_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sharework_imu_hmi_node
   ${catkin_LIBRARIES}
)

FILE(GLOB_RECURSE YamlFiles "config/*.yaml")
add_custom_target(yaml SOURCES ${YamlFiles})

FILE(GLOB_RECURSE LaunchFiles "launch/*.launch")
add_custom_target(launch SOURCES ${LaunchFiles})

