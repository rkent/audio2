cmake_minimum_required(VERSION 3.20)
project(sndplay)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C++20, needed for std::atomic with boost lockfree queue
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# Find sndfile library from debian package, it does not seem to have a CMake package
find_library(SNDFILE_LIBRARIES NAMES sndfile REQUIRED)
find_path(SNDFILE_INCLUDE_DIRS sndfile.h REQUIRED)
find_package(ALSA REQUIRED)
find_package(Boost REQUIRED)
find_package(sndplay_msgs REQUIRED)

# Add all exported headers to include/<package_name> otherwise other packages cannot see it
# This makes headers available to other packages that depend on yours.
install(DIRECTORY include/ DESTINATION include)

# 1. Define the shared library
# Add all source files of the library to add_library
add_library(
    ${PROJECT_NAME}
    SHARED
    src/alsaops.cpp
    src/buffer_file.cpp
)

#set(SOURCE_FILES
#  src/sndplay_node.cpp
#  src/alsaops.cpp
#  src/buffer_file.cpp
#  src/publish_file.cpp
#)

add_executable(publish_file src/publish_file.cpp)
add_executable(subscribe_file src/subscribe_file.cpp)
add_executable(sndplay_node src/sndplay_node.cpp)

# Set RPATH for executables to find the shared library
## TODO: This was added by claude to make it work, but should not be needed. Check why.
#set_target_properties(publish_file subscribe_file sndplay_node PROPERTIES
#  INSTALL_RPATH "$ORIGIN/../../lib"
#  BUILD_WITH_INSTALL_RPATH FALSE
#  INSTALL_RPATH_USE_LINK_PATH TRUE
#)

target_link_libraries(sndplay 
  PRIVATE
    ${SNDFILE_LIBRARIES}
    ALSA::ALSA
    rclcpp::rclcpp
    ${sndplay_msgs_TARGETS}
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(publish_file
  PRIVATE
    ${PROJECT_NAME}
    ${SNDFILE_LIBRARIES}
    ALSA::ALSA
    rclcpp::rclcpp
    ${sndplay_msgs_TARGETS}
)
target_link_libraries(subscribe_file
  PRIVATE
    ${PROJECT_NAME}
    ${SNDFILE_LIBRARIES}
    ALSA::ALSA
    rclcpp::rclcpp
    ${sndplay_msgs_TARGETS}
)

target_link_libraries(sndplay_node
  PRIVATE
    ${PROJECT_NAME}
    ${SNDFILE_LIBRARIES}
    ALSA::ALSA
    rclcpp::rclcpp
)

include_directories(${SNDFILE_INCLUDE_DIRS})

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

install(TARGETS
  ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  publish_file
  subscribe_file
  sndplay_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
