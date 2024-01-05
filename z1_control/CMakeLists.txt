cmake_minimum_required(VERSION 3.8)
project(z1_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread")
set(CMAKE_CXX_STANDARD_REQUIRED on)
set(CMAKE_CXX_STANDARD 14)
set(EIGEN_PATH /usr/include/eigen3)

# find dependencies
# find_package(ament_cmake_auto REQUIRED)
# ament_auto_find_build_dependencies()

find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pybind11 REQUIRED)
# find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc videoio)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(image_geometry REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(OpenCV REQUIRED)
# To solve: Unknown CMake command “python3_add_library”.
find_package(Python3 REQUIRED Development)
find_package(Python3 REQUIRED Interpreter)

# Include Cpp "include" directory
include_directories(
    include
    ${EIGEN_PATH}
)
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../lib)
link_directories(lib)
 
# Create Cpp executable
add_executable(cpp_test src/cpp_test.cpp)
ament_target_dependencies(cpp_test rclcpp std_msgs)

add_executable(highcmd_basic src/highcmd_basic.cpp )
target_link_libraries(highcmd_basic Z1_SDK_Linux64)
add_executable(highcmd_development src/highcmd_development.cpp )
target_link_libraries(highcmd_development Z1_SDK_Linux64)
add_executable(lowcmd_development src/lowcmd_development.cpp )
target_link_libraries(lowcmd_development Z1_SDK_Linux64)
add_executable(lowcmd_multirobots src/lowcmd_multirobots.cpp )
target_link_libraries(lowcmd_multirobots Z1_SDK_Linux64)

add_executable(demo_visual_servo src/demo_visual_servo.cpp )
ament_target_dependencies(demo_visual_servo 
  rclcpp 
  std_msgs 
  sensor_msgs 
  cv_bridge 
  image_transport 
  image_geometry
  camera_info_manager
  OpenCV)
target_link_libraries(demo_visual_servo 
  Z1_SDK_Linux64)

# Install Cpp executables
install(TARGETS
  cpp_test
  highcmd_basic
  highcmd_development
  lowcmd_development
  demo_visual_servo
  DESTINATION lib/${PROJECT_NAME}
)

# add_subdirectory(pybind11)
pybind11_add_module(unitree_arm_interface ${PROJECT_NAME}/arm_python_interface.cpp)
target_link_libraries(unitree_arm_interface PRIVATE Z1_SDK_Linux64)
# save to near .py
# install(
#   TARGETS unitree_arm_interface
#   DESTINATION ${CMAKE_BINARY_DIR}/../lib
# )

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
 
# Install Python executables
install(PROGRAMS
  scripts/z1_control_position.py
  scripts/z1_vs_velocity.py
  scripts/z1_dual_carrying.py

  scripts/example_highcmd.py
  scripts/example_lowcmd.py
  scripts/example_model.py

  DESTINATION lib/${PROJECT_NAME}
)

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

ament_package()
