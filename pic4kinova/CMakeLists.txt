cmake_minimum_required(VERSION 3.8)
project(pic4kinova)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(moveit_common REQUIRED)
moveit_package()
# added adriana
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(adriana_interfaces REQUIRED)
#find_package(moveit_core REQUIRED)
# end added adriana

set(THIS_PACKAGE_INCLUDE_DEPENDS
  moveit_ros_planning_interface
)
include_directories(${THIS_PACKAGE_INCLUDE_DIRS})
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# executables
add_executable(home src/home.cpp)
ament_target_dependencies(home
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_include_directories(home PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(home PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# added adriana
add_executable(close_gripper_adriana src/close_gripper_adriana.cpp)
ament_target_dependencies(close_gripper_adriana rclcpp moveit_task_constructor_core ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(close_gripper_adriana PUBLIC
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(close_gripper_adriana PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS
  close_gripper_adriana  
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(open_gripper_adriana src/open_gripper_adriana.cpp)
ament_target_dependencies(open_gripper_adriana rclcpp moveit_task_constructor_core ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(open_gripper_adriana PUBLIC
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(open_gripper_adriana PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS
open_gripper_adriana  
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(change_gripperStatus_server src/change_gripperStatus_server.cpp)
ament_target_dependencies(
  change_gripperStatus_server 
  rclcpp 
  std_msgs
  adriana_interfaces
  moveit_ros_planning_interface
  moveit_visual_tools
  ${THIS_PACKAGE_INCLUDE_DEPENDS} )
target_include_directories(change_gripperStatus_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
   $<INSTALL_INTERFACE:include>)
target_compile_features(change_gripperStatus_server PUBLIC c_std_99 cxx_std_17)
install(TARGETS
change_gripperStatus_server  
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(apple_picking_server src/apple_picking_server.cpp)
ament_target_dependencies(
  apple_picking_server 
  rclcpp 
  std_msgs
  geometry_msgs
  adriana_interfaces
  moveit_ros_planning_interface
  moveit_visual_tools
  ${THIS_PACKAGE_INCLUDE_DEPENDS} )
target_include_directories(apple_picking_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
   $<INSTALL_INTERFACE:include>)
target_compile_features(apple_picking_server PUBLIC c_std_99 cxx_std_17)
install(TARGETS
apple_picking_server  
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(mp_server_adriana src/mp_server_adriana.cpp)
ament_target_dependencies(
  mp_server_adriana 
  rclcpp 
  std_msgs
  geometry_msgs
  adriana_interfaces
  moveit_ros_planning_interface
  moveit_visual_tools
  ${THIS_PACKAGE_INCLUDE_DEPENDS} )
target_include_directories(mp_server_adriana PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
   $<INSTALL_INTERFACE:include>)
target_compile_features(mp_server_adriana PUBLIC c_std_99 cxx_std_17)
install(TARGETS
mp_server_adriana  
  DESTINATION lib/${PROJECT_NAME}
)



# end added by adriana

# install
install(TARGETS home
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
ament_python_install_package(${PROJECT_NAME})


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
