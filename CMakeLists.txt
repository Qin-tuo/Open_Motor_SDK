cmake_minimum_required(VERSION 3.10)
project(khcan)

set(Python3_FIND_VIRTUALENV STANDARD)
if(EXISTS "/usr/bin/python3" AND (NOT DEFINED Python3_EXECUTABLE OR Python3_EXECUTABLE MATCHES "miniconda|anaconda|conda"))
  set(Python3_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH "Python interpreter for ament" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Threads REQUIRED)

set(RMROBOT_COMMON_SRCS
  src/config_loader.cpp
  src/device.cpp
  src/mapper.cpp
  src/robot.cpp
)

add_executable(show_status
  main/show_status.cpp
  ${RMROBOT_COMMON_SRCS}
)

add_executable(test_mit_mode
  main/test_mit_mode.cpp
  ${RMROBOT_COMMON_SRCS}
)

target_include_directories(show_status PRIVATE
  include
)

target_include_directories(test_mit_mode PRIVATE
  include
)

target_link_libraries(show_status
  Threads::Threads
)

target_link_libraries(test_mit_mode
  Threads::Threads
)

ament_target_dependencies(show_status
  ament_index_cpp
)

ament_target_dependencies(test_mit_mode
  ament_index_cpp
)

install(TARGETS show_status test_mit_mode
  DESTINATION lib/${PROJECT_NAME}
)

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

ament_package()
