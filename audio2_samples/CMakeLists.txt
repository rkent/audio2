cmake_minimum_required(VERSION 3.20)
# Data only: no compiled code
project(audio2_samples NONE)

find_package(ament_cmake REQUIRED)

# Each folder is installed to share/audio2_samples/<folder>, with its license file
install(DIRECTORY kenney test_formats DESTINATION share/${PROJECT_NAME})

ament_package()
