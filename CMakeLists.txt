cmake_minimum_required(VERSION 3.10)
project(rmrobot)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yesense_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(RMROBOT_CORE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(RMROBOT_REPO_ROOT ${RMROBOT_CORE_DIR}/..)
set(RMROBOT_REMOTE_DIR ${RMROBOT_REPO_ROOT}/remote)
set(RMROBOT_IMU_DIR ${RMROBOT_REPO_ROOT}/imu)

set(RMROBOT_COMMON_INCLUDE_DIRS
  ${RMROBOT_CORE_DIR}/include
)

set(RMROBOT_COMMON_SRCS
  ${RMROBOT_CORE_DIR}/src/config_loader.cpp
  ${RMROBOT_CORE_DIR}/src/device.cpp
  ${RMROBOT_CORE_DIR}/src/mapper.cpp
  ${RMROBOT_CORE_DIR}/src/robot.cpp
)

set(RMROBOT_TARGETS)

function(rmrobot_add_target target_name entry_src)
  add_executable(${target_name} ${entry_src} ${RMROBOT_COMMON_SRCS})
  target_include_directories(${target_name} PRIVATE ${RMROBOT_COMMON_INCLUDE_DIRS})
  target_link_libraries(${target_name} Threads::Threads)

  if(ARGC GREATER 2)
    ament_target_dependencies(${target_name} ament_index_cpp ${ARGN})
  else()
    ament_target_dependencies(${target_name} ament_index_cpp)
  endif()

  list(APPEND RMROBOT_TARGETS ${target_name})
  set(RMROBOT_TARGETS "${RMROBOT_TARGETS}" PARENT_SCOPE)
endfunction()

rmrobot_add_target(show_status ${RMROBOT_CORE_DIR}/main/show_status.cpp)
rmrobot_add_target(demo_socketcan ${RMROBOT_CORE_DIR}/main/demo_socketcan.cpp yesense_interface rclcpp geometry_msgs sensor_msgs)
target_sources(demo_socketcan PRIVATE ${RMROBOT_REMOTE_DIR}/src/SbusUartReceiver.cpp)
target_include_directories(demo_socketcan PRIVATE
  ${RMROBOT_REMOTE_DIR}/include
  ${RMROBOT_IMU_DIR}/src
)

if(EXISTS ${RMROBOT_CORE_DIR}/main/main_socketcan_test.cpp)
  rmrobot_add_target(socketcan_test ${RMROBOT_CORE_DIR}/main/main_socketcan_test.cpp)
endif()

if(RMROBOT_TARGETS)
  install(TARGETS ${RMROBOT_TARGETS}
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
  install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
  )
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
  install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/setup_can_from_toml.sh)
  install(PROGRAMS
    scripts/setup_can_from_toml.sh
    DESTINATION share/${PROJECT_NAME}/scripts
  )
endif()

ament_package()
