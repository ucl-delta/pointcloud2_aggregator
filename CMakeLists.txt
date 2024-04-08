cmake_minimum_required(VERSION 3.8)
project(pointcloud2_aggregator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# This chunk is to enable the aggregator as a component
set(NODE_NAME aggregator_comp)
set(NODE_NAME_COMPONENT ${NODE_NAME}_component)
add_library(${NODE_NAME_COMPONENT} SHARED src/aggregator.cpp)
ament_target_dependencies(${NODE_NAME_COMPONENT}
  rclcpp rclcpp_components std_msgs geometry_msgs sensor_msgs)
target_include_directories(${NODE_NAME_COMPONENT} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(${NODE_NAME_COMPONENT} PRIVATE "COMPOSITION_BUILDING_DLL")
target_compile_features(${NODE_NAME_COMPONENT} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
rclcpp_components_register_node(
  ${NODE_NAME_COMPONENT}
  PLUGIN "pointcloud2::Pointcloud2_Aggregator"
  EXECUTABLE ${NODE_NAME}
)
##################
# This is the aggregator node
add_executable(aggregator src/aggregator_node.cpp)
target_link_libraries(aggregator ${NODE_NAME_COMPONENT})
ament_target_dependencies(aggregator rclcpp)

##################
# This is the monitor node
add_executable(monitor src/monitor_node.cpp)
target_link_libraries(monitor ${NODE_NAME_COMPONENT})
ament_target_dependencies(monitor rclcpp)

#######################
# This Installs the aggregator component as a runtime library
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
  )
endif()

ament_export_targets(export_${NODE_NAME_COMPONENT})
install(TARGETS ${NODE_NAME_COMPONENT}
        EXPORT export_${NODE_NAME_COMPONENT}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib
)

##########################
# This installs the aggregator and monitor node
install(TARGETS aggregator monitor
        DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/rosbag2_to_LAS.py 
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/  
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/  
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
