cmake_minimum_required(VERSION 3.8)
project(rrt_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O2)
endif()

find_package(ament_cmake      REQUIRED)
find_package(rclcpp           REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs    REQUIRED)
find_package(nav_msgs         REQUIRED)
find_package(nav2_core        REQUIRED)
find_package(nav2_costmap_2d  REQUIRED)
find_package(nav2_util        REQUIRED)
find_package(pluginlib         REQUIRED)
find_package(tf2_ros          REQUIRED)
find_package(tf2              REQUIRED)

add_library(${PROJECT_NAME} SHARED src/rrt_planner.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp rclcpp_lifecycle geometry_msgs nav_msgs
  nav2_core nav2_costmap_2d nav2_util pluginlib tf2_ros tf2)

pluginlib_export_plugin_description_file(nav2_core plugins.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/)
install(FILES plugins.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config scripts DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp rclcpp_lifecycle geometry_msgs nav_msgs
  nav2_core nav2_costmap_2d nav2_util pluginlib tf2_ros tf2)
ament_package()
