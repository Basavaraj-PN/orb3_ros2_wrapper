cmake_minimum_required(VERSION 3.5)
project(orb3_ros2_wrapper)

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

set(ORB_SLAM3_DIR
   $ENV{HOME}/ORB_SLAM3
)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


find_package(OpenCV REQUIRED)
MESSAGE("OpencvFound: ")
MESSAGE(${OpenCV_VERSION})

find_package(Eigen3 REQUIRED)
MESSAGE("Eigen3Found: ")
MESSAGE(${Eigen3_VERSION})

find_package(Pangolin REQUIRED)

include_directories(
   ${ORB_SLAM3_DIR}
   ${ORB_SLAM3_DIR}/include
   ${ORB_SLAM3_DIR}/include/CameraModels
   ${ORB_SLAM3_DIR}/Thirdparty/Sophus
   ${PROJECT_SOURCE_DIR}/include
   ${EIGEN3_INCLUDE_DIR}
   ${Pangolin_INCLUDE_DIRS}
)
set(UNIFIED_LIBS
   ${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so
   ${EIGEN3_LIBS}
   ${OpenCV_LIBS}
   ${Pangolin_LIBRARIES}
)

add_executable(
  mono_node 
  src/monocular_entry.cpp
  src/monocular_node.cpp 
  include/monocular_node.hpp
  include/adapters.hpp
  src/adapters.cpp
  include/helper_functions.hpp
  src/helper_functions.cpp
)
add_executable(camera_transform src/camera_transform_publisher.cpp)

target_include_directories(mono_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(camera_transform PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)


ament_target_dependencies(
  mono_node 
  rclcpp 
  std_msgs 
  sensor_msgs 
  tf2 
  tf2_ros 
  OpenCV 
  cv_bridge
  tf2_geometry_msgs
)

ament_target_dependencies(
  camera_transform 
  rclcpp  
  sensor_msgs 
  tf2 
  tf2_ros  
  tf2_geometry_msgs
)
target_link_libraries(mono_node ${UNIFIED_LIBS})
target_link_libraries(camera_transform ${UNIFIED_LIBS})

install(TARGETS mono_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS camera_transform
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()