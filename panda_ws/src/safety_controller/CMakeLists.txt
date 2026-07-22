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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Unit tests for the pure safety law and the lock-free buffer.
  ament_add_gtest(test_safety test/test_safety.cpp)
  target_link_libraries(test_safety safety_controller Threads::Threads)

  # Compile the standalone RT smoke harness so it never bit-rots.
  add_executable(shared_control_rt_standalone shared_control_rt.cpp)
  target_compile_definitions(shared_control_rt_standalone PRIVATE SHARED_CONTROL_RT_STANDALONE)
  target_include_directories(shared_control_rt_standalone PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(shared_control_rt_standalone Threads::Threads)
endif()

ament_package()
