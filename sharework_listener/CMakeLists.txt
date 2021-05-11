cmake_minimum_required(VERSION 3.0.2)
project(sharework_listener)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  manipulation_msgs
  roscpp
  std_msgs
  subscription_notifier
  message_generation
  message_runtime
)

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS manipulation_msgs roscpp std_msgs subscription_notifier message_generation message_runtime
  DEPENDS
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link##_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES})

add_executable(listener_node src/listener_node.cpp)
add_dependencies(listener_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(listener_node
   ${catkin_LIBRARIES}
   #${PROJECT_NAME}
)





