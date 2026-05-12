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
find_package(Threads REQUIRED)

set(KHCAN_DRIVER_SRCS
  src/config_loader.cpp
  src/device.cpp
  src/feetech_servo_device.cpp
  src/haitai_protocol.cpp
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

foreach(tool show_status test_mit_mode test_haitai_mode test_feetech_servo set_feetech_id)
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

if(BUILD_TESTING)
  add_executable(test_feetech_servo_unavailable
    tests/test_feetech_servo_unavailable.cpp
    ${KHCAN_DRIVER_SRCS}
  )
  target_include_directories(test_feetech_servo_unavailable PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_feetech_servo_unavailable
    Threads::Threads
  )
  ament_target_dependencies(test_feetech_servo_unavailable
    ament_index_cpp
  )

  add_test(NAME test_feetech_servo_unavailable
    COMMAND test_feetech_servo_unavailable
  )
endif()

install(TARGETS khcan_driver
  EXPORT export_khcan_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS show_status test_mit_mode test_haitai_mode test_feetech_servo set_feetech_id
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_targets(export_khcan_driver HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(khcan_driver)
ament_export_dependencies(Threads)

ament_package()
