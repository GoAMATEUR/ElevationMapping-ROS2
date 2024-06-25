cmake_minimum_required(VERSION 3.5)
project(elevation-mapping-ros2)

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
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_filters REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2 REQUIRED)

find_package(filters REQUIRED)
find_package(message_filters REQUIRED)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

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


set(dependencies
  grid_map_msgs
  grid_map_core
  grid_map_ros
  grid_map_filters
  rclcpp
  geometry_msgs
  tf2_eigen
  tf2_ros
  tf2
  filters
  message_filters
)
add_executable(${PROJECT_NAME}_node
  src/elevation_mapping_node.cpp
  src/ElevationMapping.cpp
  src/ElevationMap.cpp
  src/RobotMotionUpdater
  src/sensor_processing/SensorProcessorBase.cpp
  src/sensor_processing/PerfectSensorProcessor.cpp
  src/sensor_processing/StereoSensorProcessor.cpp
  src/sensor_processing/StructureLightProcessor.cpp
)

target_include_directories(${PROJECT_NAME}_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}> # Eigen3
  $<BUILD_INTERFACE:${PCL_INCLUDE_DIR}> # Eigen3
)

target_link_libraries(${PROJECT_NAME}_node
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}_node
  ${dependencies}
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

# elevation mapping componet
add_library(${PROJECT_NAME} SHARED
  src/elevation_mapping_node.cpp
  src/ElevationMapping.cpp
  src/ElevationMap.cpp
  src/RobotMotionUpdater
  src/sensor_processing/SensorProcessorBase.cpp
  src/sensor_processing/PerfectSensorProcessor.cpp
  src/sensor_processing/StereoSensorProcessor.cpp
  src/sensor_processing/StructureLightProcessor.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}> # Eigen3
  $<BUILD_INTERFACE:${PCL_INCLUDE_DIR}> # Eigen3
)

target_link_libraries(${PROJECT_NAME}
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)
rclcpp_components_register_nodes(${PROJECT_NAME} "elevation_mapping::ElevationMapping")
set(node_plugins "${node_plugins}elevation_mapping::ElevationMapping;$<TARGET_FILE:${PROJECT_NAME}>\n")
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# post processor component
add_library(${PROJECT_NAME}_post_processor SHARED
  src/post_processing/PostProcessor.cpp
)
ament_target_dependencies(${PROJECT_NAME}_post_processor
  ${dependencies}
)
target_include_directories(${PROJECT_NAME}_post_processor
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
rclcpp_components_register_nodes(${PROJECT_NAME}_post_processor "elevation_mapping::PostProcessor")
set(node_plugins "${node_plugins}elevation_mapping::PostProcessor;$<TARGET_FILE:${PROJECT_NAME}_post_processor>\n")
ament_export_targets(${PROJECT_NAME}_post_processor HAS_LIBRARY_TARGET)

install(
  TARGETS ${PROJECT_NAME}_post_processor
  EXPORT ${PROJECT_NAME}_post_processor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# node composition
add_executable(${PROJECT_NAME}_composition
  src/composition/compose_nodes.cpp
)
ament_target_dependencies(${PROJECT_NAME}_composition
  rclcpp
  rclcpp_components
)
target_link_libraries(${PROJECT_NAME}_composition
  ${PROJECT_NAME}
  ${PROJECT_NAME}_post_processor
)

target_include_directories(${PROJECT_NAME}_composition
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
install(
  TARGETS ${PROJECT_NAME}_composition
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  ${dependencies}
)
ament_package()
