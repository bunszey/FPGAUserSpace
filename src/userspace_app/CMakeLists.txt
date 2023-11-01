cmake_minimum_required(VERSION 3.5)
project(userspace_app)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

# Add your source files here
add_executable(userspace_app
	src/userspace_app.cpp
    src/xexample.c
    src/xexample_linux.c
    src/xexample_sinit.c
)

# Add the header files to the target_include_directories
target_include_directories(userspace_app
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

ament_target_dependencies(userspace_app
	rclcpp
	sensor_msgs
	OpenCV
	cv_bridge
)
install(TARGETS userspace_app DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
