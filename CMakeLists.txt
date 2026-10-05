cmake_minimum_required(VERSION 3.8)
project(icm42688_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(icm42688_driver src/icm42688_driver.cpp)
target_compile_features(icm42688_driver PUBLIC cxx_std_17)
ament_target_dependencies(icm42688_driver rclcpp sensor_msgs)

install(TARGETS icm42688_driver DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
ament_package()
