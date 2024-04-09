cmake_minimum_required(VERSION 3.8)
project(a4g16)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(world_create src/worldCreate.cpp)
#add_executable(astar_node src/Astar.cpp)
ament_target_dependencies(world_create rclcpp OpenCV sensor_msgs)

#include_directories(include)
#include_directories(${sensor_msgs_INCLUDE_DIRS})
#target_link_libraries(astar_node ${sensor_msgs_LIBRARIES})
#include_directories(${OpenCV_INCLUDE_DIRS})
#target_link_libraries(astar_node ${OpenCV_LIBRARIES})


#target_compile_features(my_executable PRIVATE cxx_std_11)
# Include launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS world_create
        DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
    worlds
  DESTINATION share/${PROJECT_NAME}
)


# Install Python files
install(FILES scripts/Astar.py DESTINATION bin)

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
ament_export_dependencies(rclcpp OpenCV sensor_msgs)
ament_package()
