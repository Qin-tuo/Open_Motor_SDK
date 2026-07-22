cmake_minimum_required(VERSION 3.10)
project(khcan VERSION 0.1.0)

set(Python3_FIND_VIRTUALENV STANDARD)
if(EXISTS "/usr/bin/python3" AND
   (NOT DEFINED Python3_EXECUTABLE OR Python3_EXECUTABLE MATCHES "miniconda|anaconda|conda"))
  set(Python3_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH "Python interpreter for ament" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Threads REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorStatus.msg"
  "msg/MotorStatusArray.msg"
  "srv/SetMotorMode.srv"
  DEPENDENCIES std_msgs
)

set(KHCAN_DRIVER_SRCS
  src/config_loader.cpp
  src/device.cpp
  src/mapper.cpp
  src/robot.cpp
)

add_library(khcan_driver SHARED
  ${KHCAN_DRIVER_SRCS}
)

target_include_directories(khcan_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(khcan_driver PUBLIC
  Threads::Threads
)

set(KHCAN_TOOL_TARGETS
  show_status
  test_mit_mode
)

foreach(tool ${KHCAN_TOOL_TARGETS})
  add_executable(${tool}
    main/${tool}.cpp
  )
  target_link_libraries(${tool}
    khcan_driver
  )
  ament_target_dependencies(${tool}
    ament_index_cpp
  )
endforeach()

add_executable(motor_driver_node
  main/motor_driver_node.cpp
)
target_link_libraries(motor_driver_node
  khcan_driver
)
ament_target_dependencies(motor_driver_node
  ament_index_cpp
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(motor_driver_node
  "${cpp_typesupport_target}"
)

install(TARGETS khcan_driver
  EXPORT export_khcan_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS ${KHCAN_TOOL_TARGETS}
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS motor_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_targets(export_khcan_driver HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(khcan_driver)
ament_export_dependencies(rosidl_default_runtime Threads)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_driver_core test/test_driver_core.cpp)
  target_link_libraries(test_driver_core khcan_driver)
  add_test(NAME driver_core COMMAND test_driver_core ${CMAKE_CURRENT_SOURCE_DIR}/config/motor.toml)
  add_test(NAME config_m4 COMMAND test_driver_core ${CMAKE_CURRENT_SOURCE_DIR}/config/m4.toml)
  add_test(NAME config_k1 COMMAND test_driver_core ${CMAKE_CURRENT_SOURCE_DIR}/config/k1.toml)
endif()

ament_package()
