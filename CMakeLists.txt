cmake_minimum_required(VERSION 3.10)
project(dingcan)

# =========================================================
# 1. 查找依赖
# =========================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# 【修复 1】添加 Threads 库查找，解决 Threads::Threads 报错
find_package(Threads REQUIRED)

# =========================================================
# 2. 基础设置
# =========================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

# =========================================================
# 3. 编译底层驱动库 (libdingcan_driver)
# =========================================================
file(GLOB_RECURSE ALL_SRCS "src/*.cpp")

add_library(dingcan_driver SHARED ${ALL_SRCS})

target_include_directories(dingcan_driver PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 链接 USB-CAN 库
find_library(USB_CAN_LIB usb_can PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib NO_DEFAULT_PATH)

# 这里链接 Threads::Threads 现在是安全的，因为上面添加了 find_package
target_link_libraries(dingcan_driver PUBLIC ${USB_CAN_LIB} Threads::Threads)

# =========================================================
# 4. 编译示例程序 (Executables)
# =========================================================
add_executable(lingzu_robot main/main_exec.cpp)
add_executable(show_status main/show_status.cpp)

target_include_directories(lingzu_robot PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_include_directories(show_status PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(lingzu_robot PRIVATE dingcan_driver)
target_link_libraries(show_status PRIVATE dingcan_driver)

# =========================================================
# 5. 安装规则
# =========================================================
install(DIRECTORY include/ DESTINATION include)

install(TARGETS dingcan_driver lingzu_robot show_status
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/lib/libusb_can.so DESTINATION lib)

ament_export_include_directories(include)
ament_export_libraries(dingcan_driver)
ament_package()
