cmake_minimum_required(VERSION 3.8)
project(ros2_serialsample2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sft_interfaces REQUIRED)

set(pub2_sources
src/pubsample2.cpp
src/serial.cpp
include/commonsft.hpp
include/serial.hpp
)

set(sub2_sources
src/subsample2.cpp
)

add_executable(pubsample2 ${pub2_sources})
add_executable(subsample2 ${sub2_sources})

include_directories(${sft_interfaces_INCLUDE_DIRS})
target_include_directories(pubsample2 PRIVATE ./include)
target_include_directories(subsample2 PRIVATE ./include)

ament_target_dependencies(pubsample2 rclcpp std_msgs sft_interfaces)
ament_target_dependencies(subsample2 rclcpp std_msgs sft_interfaces)

install(TARGETS
    pubsample2
    subsample2
    DESTINATION lib/${PROJECT_NAME})

ament_package()
