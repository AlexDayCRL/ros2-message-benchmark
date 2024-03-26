cmake_minimum_required(VERSION 3.5)
project(ros2_messaging)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(spdlog REQUIRED)

include_directories(include)
ament_export_include_directories(include)

add_executable(talker src/talker.cpp)
target_link_libraries(talker
  spdlog::spdlog
)
ament_target_dependencies(talker
  "rclcpp"
  "sensor_msgs"
)

install(TARGETS
  talker
  DESTINATION bin/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
