cmake_minimum_required(VERSION 3.8)
project(safety_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)

add_library(safety_controller
  shared_control_rt.cpp
)

target_include_directories(safety_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(safety_controller
  Threads::Threads
)

install(
  TARGETS safety_controller
  EXPORT export_safety_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(safety_controller)
ament_package()
