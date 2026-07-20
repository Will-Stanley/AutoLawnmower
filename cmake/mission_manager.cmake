###
# mission_manager
###
add_executable(mission_manager
  src/mission_manager/main.cpp
  src/mission_manager/mission_manager_node.cpp
)
target_compile_features(mission_manager PUBLIC c_std_99 cxx_std_17)
target_include_directories(mission_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mission_manager
  "${cpp_typesupport_target}")
ament_target_dependencies(mission_manager
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  sensor_msgs
  nav_msgs
  nav2_msgs
  tf2
  tf2_geometry_msgs
)

INSTALL(TARGETS mission_manager
        DESTINATION lib/${PROJECT_NAME})

add_dependencies(mission_manager ${PROJECT_NAME})
