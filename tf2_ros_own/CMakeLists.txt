cmake_minimum_required(VERSION 3.5)
project(tf2_ros_own)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT ANDROID)
  set(TF2_PY tf2_py)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)

include_directories(include
  ${geometry_msgs_INCLUDE_DIRS}
  ${message_filters_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_msgs_INCLUDE_DIRS}
)

# tf2_ros_own library
add_library(${PROJECT_NAME} SHARED
  src/buffer.cpp
  src/create_timer_ros.cpp
  src/transform_listener.cpp
  # src/buffer_client.cpp
  # src/buffer_server.cpp
  src/transform_broadcaster.cpp
  src/static_transform_broadcaster.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${geometry_msgs_LIBRARIES}
  ${message_filters_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_msgs_LIBRARIES}
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "TF2_ROS_BUILDING_DLL")

add_executable(transform
  src/transform.cpp
)
ament_target_dependencies(transform
  "custom"
)
target_link_libraries(transform
  ${PROJECT_NAME}
  ${geometry_msgs_LIBRARIES}
  ${custom_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_msgs_LIBRARIES}
)

# Install rules
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install executables
install(TARGETS
  transform
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(message_filters)
ament_export_dependencies(rclcpp)
ament_export_dependencies(tf2_msgs)
ament_export_dependencies(tf2)
ament_package()
