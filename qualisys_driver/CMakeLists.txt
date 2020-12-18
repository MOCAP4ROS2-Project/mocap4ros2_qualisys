cmake_minimum_required(VERSION 3.5)

project(qualisys_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_SOURCE_DIR}/cmake/modules")
find_package(QUALISYSSDK REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(mocap_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


set(dependencies
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_ros
  mocap_msgs
)

include_directories(
  include
  ${QUALISYSSDK_INCLUDE_DIR}
)

add_library(${PROJECT_NAME} STATIC src/qualisys_driver.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_library(qualisys_cpp_sdk STATIC IMPORTED)
set_target_properties(qualisys_cpp_sdk PROPERTIES
    IMPORTED_LOCATION ${QUALISYSSDK_LIBRARY})

add_executable(qualisys_driver_main
  src/qualisys_driver_main.cpp
)

ament_target_dependencies(qualisys_driver_main ${dependencies})
target_link_libraries(qualisys_driver_main
  ${PROJECT_NAME}
  qualisys_cpp_sdk
)

ament_target_dependencies(qualisys_driver ${dependencies})
target_link_libraries(qualisys_driver
  qualisys_cpp_sdk
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  qualisys_driver_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
