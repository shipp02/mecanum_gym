cmake_minimum_required(VERSION 3.8)
project(mecanum)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_control_test_assets REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(serial_driver REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(mecanum_hf SHARED
  src/ias_mecanum.cpp)


# ament_uncrustify_format_source_files(
#        DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
#        TARGET_NAME mecanum_hf
# )

target_include_directories(mecanum_hf
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(mecanum_hf hardware_interface rclcpp serial_driver io_context)
pluginlib_export_plugin_description_file(hardware_interface ias_mecanum.xml)

ament_add_gmock(mecanum_test test/test_load_ias_mecanum.cpp)

target_include_directories(mecanum_test
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(mecanum_test hardware_interface rclcpp ros2_control_test_assets)

install(TARGETS mecanum_hf
  EXPORT export_mecanum_hf
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
