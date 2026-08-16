cmake_minimum_required(VERSION 3.8)
project(usv_radio_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# No other package in this workspace sets CXX_STANDARD (Humble defaults to
# C++14); this one needs 17 (std::optional, std::clamp, structured-binding-
# free but still C++17-only bits elsewhere), so it's set explicitly here and
# only here.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(usv_gnc_msgs REQUIRED)
find_package(ZLIB REQUIRED)
find_package(Threads REQUIRED)

set(lib_name ${PROJECT_NAME}_lib)

add_library(${lib_name}
  src/protocol.cpp
  src/crc.cpp
  src/frame_parser.cpp
  src/serial_transport.cpp
  src/radio_bridge_node.cpp
)

target_include_directories(${lib_name} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${lib_name}
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nmea_msgs
  mavros_msgs
  usv_gnc_msgs
)

# CRC32 must byte-for-byte match Python's zlib.crc32 output (see crc.cpp) —
# link system zlib directly rather than hand-rolling the table.
target_link_libraries(${lib_name} ZLIB::ZLIB Threads::Threads)

add_executable(radio_bridge_node src/main.cpp)
target_link_libraries(radio_bridge_node ${lib_name})
ament_target_dependencies(radio_bridge_node rclcpp)

install(TARGETS ${lib_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS radio_bridge_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_export_include_directories(include)
ament_export_libraries(${lib_name})
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nmea_msgs
  mavros_msgs
  usv_gnc_msgs
)

ament_package()
