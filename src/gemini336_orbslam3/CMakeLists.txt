cmake_minimum_required(VERSION 3.8)
project(gemini336_orbslam3)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Resolve ROS integration and the libraries exposed by the existing SLAM build.
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
find_package(spdlog REQUIRED)

# Resolve the workspace at configure time, never from a node working directory.
get_filename_component(_workspace_root "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
add_library(gemini336_logging STATIC src/logging.cpp)
target_compile_features(gemini336_logging PUBLIC cxx_std_17)
target_include_directories(gemini336_logging PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_compile_definitions(gemini336_logging PRIVATE
  GEMINI336_DEFAULT_LOG_DIRECTORY="${_workspace_root}/log_slam"
)
target_link_libraries(gemini336_logging PUBLIC spdlog::spdlog)

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

# Describe the prebuilt backend without rebuilding third-party sources here.
add_library(ORB_SLAM3::ORB_SLAM3 SHARED IMPORTED)
set_target_properties(ORB_SLAM3::ORB_SLAM3 PROPERTIES
  IMPORTED_LOCATION "${ORB_SLAM3_ROOT}/lib/libORB_SLAM3.so"
  INTERFACE_INCLUDE_DIRECTORIES
    "${ORB_SLAM3_ROOT};${ORB_SLAM3_ROOT}/include;${ORB_SLAM3_ROOT}/include/CameraModels;${ORB_SLAM3_ROOT}/Thirdparty/Sophus;${Pangolin_INCLUDE_DIRS};${OpenCV_INCLUDE_DIRS}"
  INTERFACE_LINK_LIBRARIES "Eigen3::Eigen;${Pangolin_LIBRARIES};${OpenCV_LIBS};spdlog::spdlog"
)

# Keep backend adaptation separate from ROS sensor subscriptions.
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

# The input checker exercises the frontend without starting SLAM.
add_executable(stereo_frontend_check_node tools/stereo_frontend_check_node.cpp)
target_link_libraries(stereo_frontend_check_node stereo_frontend)
ament_target_dependencies(stereo_frontend_check_node rclcpp)

add_executable(imu_frontend_check_node tools/imu_frontend_check_node.cpp)
target_link_libraries(imu_frontend_check_node imu_frontend)
ament_target_dependencies(imu_frontend_check_node rclcpp)

add_library(tracking_coordinator STATIC src/slam/tracking_coordinator.cpp)
target_compile_features(tracking_coordinator PUBLIC cxx_std_17)
target_include_directories(tracking_coordinator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(tracking_coordinator PUBLIC orbslam3_adapter imu_frontend gemini336_logging)

add_executable(slam_node src/slam_node.cpp)
target_compile_features(slam_node PRIVATE cxx_std_17)
target_compile_definitions(slam_node PRIVATE
  ORB_SLAM3_DEFAULT_VOCABULARY_PATH="${ORB_SLAM3_ROOT}/Vocabulary/ORBvoc.txt"
)
target_link_libraries(slam_node tracking_coordinator orbslam3_adapter stereo_frontend imu_frontend gemini336_logging)
ament_target_dependencies(slam_node rclcpp)

# Install runnable nodes and their ROS parameter defaults.
install(TARGETS stereo_frontend_check_node imu_frontend_check_node slam_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(Threads REQUIRED)
  add_library(tracking_coordinator_test STATIC src/slam/tracking_coordinator.cpp)
  target_compile_features(tracking_coordinator_test PUBLIC cxx_std_17)
  target_compile_definitions(tracking_coordinator_test PUBLIC GEMINI336_QUEUE_TEST)
  target_include_directories(tracking_coordinator_test PUBLIC include)
  target_link_libraries(tracking_coordinator_test PUBLIC orbslam3_adapter imu_frontend gemini336_logging)
  add_executable(slam_queue_test test/slam_queue_test.cpp)
  target_compile_features(slam_queue_test PRIVATE cxx_std_17)
  target_compile_definitions(slam_queue_test PRIVATE
    GEMINI336_QUEUE_TEST
    ORB_SLAM3_DEFAULT_VOCABULARY_PATH="${ORB_SLAM3_ROOT}/Vocabulary/ORBvoc.txt"
  )
  target_link_libraries(slam_queue_test
    tracking_coordinator_test orbslam3_adapter stereo_frontend imu_frontend gemini336_logging Threads::Threads)
  ament_target_dependencies(slam_queue_test rclcpp)
  add_test(NAME slam_queue_test COMMAND slam_queue_test)
  set_tests_properties(slam_queue_test PROPERTIES TIMEOUT 30)
  # Compile the real coordination path with only the backend replaced for finite tests.
  add_executable(slam_tracking_test test/slam_tracking_test.cpp)
  target_compile_features(slam_tracking_test PRIVATE cxx_std_17)
  target_compile_definitions(slam_tracking_test PRIVATE
    GEMINI336_QUEUE_TEST
    ORB_SLAM3_DEFAULT_VOCABULARY_PATH="${ORB_SLAM3_ROOT}/Vocabulary/ORBvoc.txt"
  )
  target_link_libraries(slam_tracking_test
    tracking_coordinator_test orbslam3_adapter stereo_frontend imu_frontend gemini336_logging Threads::Threads)
  ament_target_dependencies(slam_tracking_test rclcpp)
  add_test(NAME slam_tracking_test COMMAND slam_tracking_test)
  set_tests_properties(slam_tracking_test PROPERTIES TIMEOUT 30)
  # Bounded executor integration uses production frontend/timer wiring and a fake backend.
  add_executable(slam_executor_test test/slam_executor_test.cpp)
  target_compile_features(slam_executor_test PRIVATE cxx_std_17)
  target_compile_definitions(slam_executor_test PRIVATE
    GEMINI336_QUEUE_TEST
    ORB_SLAM3_DEFAULT_VOCABULARY_PATH="${ORB_SLAM3_ROOT}/Vocabulary/ORBvoc.txt"
  )
  target_link_libraries(slam_executor_test
    tracking_coordinator_test orbslam3_adapter stereo_frontend imu_frontend gemini336_logging Threads::Threads)
  ament_target_dependencies(slam_executor_test rclcpp)
  add_test(NAME slam_executor_test COMMAND slam_executor_test)
  set_tests_properties(slam_executor_test PROPERTIES TIMEOUT 45)
  # Exercise actual adapter lifecycle code without constructing an ORB-SLAM3 System.
  add_executable(process_finalization_test test/process_finalization_test.cpp)
  target_compile_features(process_finalization_test PRIVATE cxx_std_17)
  target_include_directories(process_finalization_test PRIVATE include)
  target_link_libraries(process_finalization_test Threads::Threads)
  add_test(NAME process_finalization_test COMMAND process_finalization_test)
  set_tests_properties(process_finalization_test PROPERTIES TIMEOUT 30)
  add_executable(adapter_shutdown_test
    test/adapter_shutdown_test.cpp
    test/orbslam3_adapter_test_support.cpp
    src/slam/orbslam3_adapter.cpp)
  target_compile_features(adapter_shutdown_test PRIVATE cxx_std_17)
  target_compile_definitions(adapter_shutdown_test PRIVATE GEMINI336_ADAPTER_TEST)
  target_include_directories(adapter_shutdown_test PRIVATE include)
  target_link_libraries(adapter_shutdown_test ORB_SLAM3::ORB_SLAM3 Threads::Threads)
  add_test(NAME adapter_shutdown_test COMMAND adapter_shutdown_test)
  set_tests_properties(adapter_shutdown_test PROPERTIES TIMEOUT 30)
  # Idle worker lifecycle only: no camera, map, or ORB-SLAM3 System construction.
  add_executable(local_mapping_shutdown_test test/local_mapping_shutdown_test.cpp)
  target_compile_features(local_mapping_shutdown_test PRIVATE cxx_std_17)
  # This test directly destroys Eigen members allocated by the backend, which is
  # built with -march=native. Match its alignment/allocation ABI at both ends.
  target_compile_options(local_mapping_shutdown_test PRIVATE -march=native)
  target_link_libraries(local_mapping_shutdown_test ORB_SLAM3::ORB_SLAM3 Threads::Threads)
  add_test(NAME local_mapping_shutdown_test COMMAND local_mapping_shutdown_test)
  set_tests_properties(local_mapping_shutdown_test PROPERTIES TIMEOUT 15)
  # Fake GBA work exercises real lifecycle ownership without executing an optimizer.
  add_executable(loop_closing_shutdown_test test/loop_closing_shutdown_test.cpp)
  target_compile_features(loop_closing_shutdown_test PRIVATE cxx_std_17)
  # Match the backend's Eigen layout/allocation ABI for this direct core-object test.
  target_compile_options(loop_closing_shutdown_test PRIVATE -march=native)
  target_link_libraries(loop_closing_shutdown_test ORB_SLAM3::ORB_SLAM3 Threads::Threads)
  add_test(NAME loop_closing_shutdown_test COMMAND loop_closing_shutdown_test)
  set_tests_properties(loop_closing_shutdown_test PROPERTIES TIMEOUT 15)
  # Exercise the exact backend coordination class without maps or an optimizer.
  add_executable(tracking_imu_shutdown_test test/tracking_imu_shutdown_test.cpp)
  target_compile_features(tracking_imu_shutdown_test PRIVATE cxx_std_17)
  target_include_directories(tracking_imu_shutdown_test PRIVATE "${ORB_SLAM3_ROOT}/include")
  target_link_libraries(tracking_imu_shutdown_test Threads::Threads)
  add_test(NAME tracking_imu_shutdown_test COMMAND tracking_imu_shutdown_test)
  set_tests_properties(tracking_imu_shutdown_test PROPERTIES TIMEOUT 15)
  # Headless lifecycle/notification coordination only; no GUI or SLAM runtime.
  add_executable(viewer_shutdown_test test/viewer_shutdown_test.cpp)
  target_compile_features(viewer_shutdown_test PRIVATE cxx_std_17)
  target_include_directories(viewer_shutdown_test PRIVATE include "${ORB_SLAM3_ROOT}/include")
  target_link_libraries(viewer_shutdown_test Threads::Threads)
  add_test(NAME viewer_shutdown_test COMMAND viewer_shutdown_test)
  set_tests_properties(viewer_shutdown_test PROPERTIES TIMEOUT 15)
  add_executable(context_stop_registration_test test/context_stop_registration_test.cpp)
  target_compile_features(context_stop_registration_test PRIVATE cxx_std_17)
  target_include_directories(context_stop_registration_test PRIVATE include)
  target_link_libraries(context_stop_registration_test Threads::Threads)
  ament_target_dependencies(context_stop_registration_test rclcpp)
  add_test(NAME context_stop_registration_test COMMAND context_stop_registration_test)
  set_tests_properties(context_stop_registration_test PROPERTIES TIMEOUT 30)
  add_executable(callback_guard_test test/callback_guard_test.cpp)
  target_compile_features(callback_guard_test PRIVATE cxx_std_17)
  target_include_directories(callback_guard_test PRIVATE include)
  target_link_libraries(callback_guard_test Threads::Threads)
  add_test(NAME callback_guard_test COMMAND callback_guard_test)
  set_tests_properties(callback_guard_test PROPERTIES TIMEOUT 30)
  add_executable(stop_control_test test/stop_control_test.cpp)
  target_compile_features(stop_control_test PRIVATE cxx_std_17)
  target_include_directories(stop_control_test PRIVATE include)
  target_link_libraries(stop_control_test Threads::Threads)
  add_test(NAME stop_control_test COMMAND stop_control_test)
  set_tests_properties(stop_control_test PROPERTIES TIMEOUT 30)
  add_executable(diagnostic_trace_test test/diagnostic_trace_test.cpp)
  target_compile_features(diagnostic_trace_test PRIVATE cxx_std_17)
  target_include_directories(diagnostic_trace_test PRIVATE include)
  target_link_libraries(diagnostic_trace_test Threads::Threads)
  add_test(NAME diagnostic_trace_test COMMAND diagnostic_trace_test)
  set_tests_properties(diagnostic_trace_test PROPERTIES TIMEOUT 30)
  add_executable(imu_frontend_test test/imu_frontend_test.cpp)
  target_link_libraries(imu_frontend_test imu_frontend)
  ament_target_dependencies(imu_frontend_test rclcpp sensor_msgs)
  add_test(NAME imu_frontend_test COMMAND imu_frontend_test)
  set_tests_properties(imu_frontend_test PROPERTIES TIMEOUT 30)
  add_executable(logging_test test/logging_test.cpp)
  target_link_libraries(logging_test gemini336_logging)
  add_test(NAME logging_test COMMAND logging_test)
  # Core logging contract only: no System construction, sensors, GUI or optimizer.
  add_executable(orbslam3_logging_test test/orbslam3_logging_test.cpp)
  target_include_directories(orbslam3_logging_test PRIVATE "${ORB_SLAM3_ROOT}/include")
  target_link_libraries(orbslam3_logging_test gemini336_logging Threads::Threads)
  add_test(NAME orbslam3_logging_test COMMAND orbslam3_logging_test)
  set_tests_properties(orbslam3_logging_test PROPERTIES TIMEOUT 30)
  # Settings and empty map logging/serialization only; no System or worker launch.
  add_executable(settings_map_logging_test test/settings_map_logging_test.cpp)
  target_compile_options(settings_map_logging_test PRIVATE -march=native)
  target_link_libraries(settings_map_logging_test
    gemini336_logging ORB_SLAM3::ORB_SLAM3 boost_serialization
    "${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib/libDBoW2.so")
  add_test(NAME settings_map_logging_test COMMAND settings_map_logging_test)
  set_tests_properties(settings_map_logging_test PROPERTIES TIMEOUT 30)
  # Real worker diagnostics and finite fake GBA work; no sensors or System launch.
  add_executable(worker_logging_test test/worker_logging_test.cpp)
  target_compile_options(worker_logging_test PRIVATE -march=native)
  target_link_libraries(worker_logging_test
    gemini336_logging ORB_SLAM3::ORB_SLAM3 Threads::Threads
    "${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib/libDBoW2.so")
  add_test(NAME worker_logging_test COMMAND worker_logging_test)
  set_tests_properties(worker_logging_test PROPERTIES TIMEOUT 30)
  # Viewer/MapDrawer configuration diagnostics only; no GUI or System launch.
  add_executable(viewer_logging_test test/viewer_logging_test.cpp)
  target_compile_options(viewer_logging_test PRIVATE -march=native)
  target_link_libraries(viewer_logging_test gemini336_logging ORB_SLAM3::ORB_SLAM3)
  add_test(NAME viewer_logging_test COMMAND viewer_logging_test)
  set_tests_properties(viewer_logging_test PROPERTIES TIMEOUT 30)
  # Local BA diagnostic early return only; no numerical optimization or sensors.
  add_executable(optimizer_logging_test test/optimizer_logging_test.cpp)
  target_compile_options(optimizer_logging_test PRIVATE -march=native)
  target_link_libraries(optimizer_logging_test
    gemini336_logging ORB_SLAM3::ORB_SLAM3
    "${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib/libDBoW2.so")
  add_test(NAME optimizer_logging_test COMMAND optimizer_logging_test)
  set_tests_properties(optimizer_logging_test PROPERTIES TIMEOUT 30)
  # Data-object diagnostic branches only; no sensors, cameras or worker launch.
  add_executable(data_logging_test test/data_logging_test.cpp)
  target_compile_options(data_logging_test PRIVATE -march=native)
  target_link_libraries(data_logging_test
    gemini336_logging ORB_SLAM3::ORB_SLAM3 boost_serialization
    "${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib/libDBoW2.so")
  add_test(NAME data_logging_test COMMAND data_logging_test)
  set_tests_properties(data_logging_test PROPERTIES TIMEOUT 30)
  # Matrix checks, IMU reports and legacy verbosity only; no SLAM runtime launch.
  add_executable(utility_logging_test test/utility_logging_test.cpp)
  target_compile_options(utility_logging_test PRIVATE -march=native)
  target_link_libraries(utility_logging_test gemini336_logging ORB_SLAM3::ORB_SLAM3)
  add_test(NAME utility_logging_test COMMAND utility_logging_test)
  set_tests_properties(utility_logging_test PROPERTIES TIMEOUT 30)
  # Synthetic scalar LM and IMU metadata fixtures; no System, sensors or SLAM run.
  add_executable(gba_diagnostics_test test/gba_diagnostics_test.cpp)
  target_compile_options(gba_diagnostics_test PRIVATE -march=native)
  target_link_libraries(gba_diagnostics_test
    gemini336_logging ORB_SLAM3::ORB_SLAM3 Threads::Threads
    "${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib/libg2o.so")
  add_test(NAME gba_diagnostics_test COMMAND gba_diagnostics_test)
  set_tests_properties(gba_diagnostics_test PROPERTIES TIMEOUT 30)
  add_executable(process_evidence_test test/process_evidence_test.cpp)
  target_compile_features(process_evidence_test PRIVATE cxx_std_17)
  target_include_directories(process_evidence_test PRIVATE include)
  target_link_libraries(process_evidence_test Threads::Threads)
  add_test(NAME process_evidence_test COMMAND process_evidence_test)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME validation_common_test COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/validation_common_test.py)
  set_tests_properties(process_evidence_test validation_common_test PROPERTIES TIMEOUT 30)
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
