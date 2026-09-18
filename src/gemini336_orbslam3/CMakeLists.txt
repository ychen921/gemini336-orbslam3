cmake_minimum_required(VERSION 3.8)
project(gemini336_orbslam3)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV 4.4 REQUIRED COMPONENTS core)
find_package(Eigen3 3.1 REQUIRED NO_MODULE)
find_package(Pangolin REQUIRED)

get_filename_component(_orbslam3_default_root
  "${CMAKE_CURRENT_SOURCE_DIR}/../../external/ORB_SLAM3" ABSOLUTE)
set(ORB_SLAM3_ROOT "${_orbslam3_default_root}" CACHE PATH
  "Root of the existing ORB-SLAM3 source and build tree")

# Use the existing build; do not pull upstream examples into the ROS package.
foreach(_orbslam3_required_path
    include/System.h include/CameraModels Thirdparty/Sophus/sophus/se3.hpp
    lib/libORB_SLAM3.so)
  if(NOT EXISTS "${ORB_SLAM3_ROOT}/${_orbslam3_required_path}")
    message(FATAL_ERROR
      "Missing ${ORB_SLAM3_ROOT}/${_orbslam3_required_path}. "
      "Set ORB_SLAM3_ROOT to an ORB-SLAM3 tree built inside the Docker workspace.")
  endif()
endforeach()

add_library(ORB_SLAM3::ORB_SLAM3 SHARED IMPORTED)
set_target_properties(ORB_SLAM3::ORB_SLAM3 PROPERTIES
  IMPORTED_LOCATION "${ORB_SLAM3_ROOT}/lib/libORB_SLAM3.so"
  INTERFACE_INCLUDE_DIRECTORIES
    "${ORB_SLAM3_ROOT};${ORB_SLAM3_ROOT}/include;${ORB_SLAM3_ROOT}/include/CameraModels;${ORB_SLAM3_ROOT}/Thirdparty/Sophus;${Pangolin_INCLUDE_DIRS};${OpenCV_INCLUDE_DIRS}"
  INTERFACE_LINK_LIBRARIES "Eigen3::Eigen;${Pangolin_LIBRARIES};${OpenCV_LIBS}"
)

add_library(orbslam3_adapter STATIC src/slam/orbslam3_adapter.cpp)
target_compile_features(orbslam3_adapter PUBLIC cxx_std_17)
target_include_directories(orbslam3_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(orbslam3_adapter
  PUBLIC ${OpenCV_LIBS} Eigen3::Eigen
  PRIVATE ORB_SLAM3::ORB_SLAM3
)

add_library(stereo_frontend STATIC src/frontend/stereo_frontend.cpp)
target_compile_features(stereo_frontend PUBLIC cxx_std_17)
target_include_directories(stereo_frontend PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(stereo_frontend
  rclcpp sensor_msgs message_filters cv_bridge
)
target_link_libraries(stereo_frontend Eigen3::Eigen)

add_library(imu_frontend STATIC src/frontend/imu_frontend.cpp)
target_compile_features(imu_frontend PUBLIC cxx_std_17)
target_include_directories(imu_frontend PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(imu_frontend rclcpp sensor_msgs)
target_link_libraries(imu_frontend Eigen3::Eigen ${OpenCV_LIBS})

add_executable(stereo_frontend_check_node tools/stereo_frontend_check_node.cpp)
target_link_libraries(stereo_frontend_check_node stereo_frontend)
ament_target_dependencies(stereo_frontend_check_node rclcpp)
add_executable(slam_node src/slam_node.cpp)
target_compile_features(slam_node PRIVATE cxx_std_17)
target_compile_definitions(slam_node PRIVATE
  ORB_SLAM3_DEFAULT_VOCABULARY_PATH="${ORB_SLAM3_ROOT}/Vocabulary/ORBvoc.txt"
)
target_link_libraries(slam_node orbslam3_adapter stereo_frontend)
ament_target_dependencies(slam_node rclcpp)

install(TARGETS stereo_frontend_check_node slam_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

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
