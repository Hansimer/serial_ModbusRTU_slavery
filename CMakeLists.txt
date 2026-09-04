cmake_minimum_required(VERSION 3.8)
project(serial_modbus_rtu_slavery)

# 强制使用 C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O0 -ggdb")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -ggdb")
endif()

# find ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(my_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Threads REQUIRED)

# ========== 手动查找 libmodbus（参考 robot_ctrol_node 的查找方式）==========
find_path(MODBUS_INC_DIR NAMES modbus/modbus.h)
find_library(MODBUS_LIB NAMES modbus)
if(NOT MODBUS_INC_DIR OR NOT MODBUS_LIB)
  message(FATAL_ERROR "libmodbus not found, please install libmodbus-dev")
endif()

include_directories(include ${MODBUS_INC_DIR})

# 可执行文件
add_executable(serial_modbus_rtu_slavery_node
  src/serial_modbus_rtu_slavery_node.cpp
  src/led_protocol.cpp
  src/modbus_master.cpp
)

target_link_libraries(serial_modbus_rtu_slavery_node
  ${MODBUS_LIB}
  Threads::Threads
)

ament_target_dependencies(serial_modbus_rtu_slavery_node
  rclcpp
  my_interfaces
  ament_index_cpp
)

# 安装可执行文件
install(TARGETS
  serial_modbus_rtu_slavery_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装 config 参数文件
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# 安装 log 目录
install(DIRECTORY log
  DESTINATION log/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

# ============================================================
# CPack: 直接从 CMakeLists.txt 生成 .deb 安装包
# 注意: 所有 set(CPACK_*) 必须写在 include(CPack) 之前
# 用法:在文件src/serial_modbus_rtu_slavery中执行
#   cmake -S . -B build-cpack -DCMAKE_INSTALL_PREFIX=/opt/ros/humble -DBUILD_TESTING=OFF
#   cmake --build build-cpack
#   cpack --config build-cpack/CPackConfig.cmake -G DEB
#   生成的 deb 在 build-cpack/ 目录下, 用 sudo dpkg -i <deb> 安装
# ============================================================
# 包名/版本/维护者 (Debian 规范: 包名带 ros-humble- 前缀)
set(CPACK_PACKAGE_NAME            "embody_serial_modbus_rtu_slavery")
set(CPACK_PACKAGE_VERSION         "1.0.0")
set(CPACK_PACKAGE_CONTACT         "hsm <137196922@qq.com>")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "hsm <137196922@qq.com>")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "serial_modbus_rtu_slavery: Modbus RTU slavery for ROS2")
# 安装到系统 ROS2 前缀, 与 bloom 生成的包路径一致
set(CPACK_PACKAGING_INSTALL_PREFIX "/opt/ros/humble")
# 生成 DEB 安装包
set(CPACK_GENERATOR                "DEB")
# aarch64 架构
set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "arm64")
# 运行时依赖 (dpkg -i 不自动解析依赖, 需保证这些包已安装)
set(CPACK_DEBIAN_PACKAGE_DEPENDS
    "libc6, libgcc-s1, libpython3.10, libstdc++6, \
ros-humble-action-msgs, ros-humble-builtin-interfaces, ros-humble-fastcdr, \
ros-humble-rosidl-default-runtime, ros-humble-sensor-msgs, \
ros-humble-unique-identifier-msgs")

set(CPACK_PACKAGE_FILE_NAME       "${CPACK_PACKAGE_NAME}_${CPACK_PACKAGE_VERSION}_${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")
include(CPack)
