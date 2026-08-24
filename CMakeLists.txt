cmake_minimum_required(VERSION 3.8)
project(serial_ModbusRTU_slavery)

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
add_executable(serial_ModbusRTU_slavery_node
  src/serial_ModbusRTU_slavery_node.cpp
  src/led_protocol.cpp
  src/modbus_master.cpp
)

target_link_libraries(serial_ModbusRTU_slavery_node
  ${MODBUS_LIB}
  Threads::Threads
)

ament_target_dependencies(serial_ModbusRTU_slavery_node
  rclcpp
  my_interfaces
  ament_index_cpp
)

# 安装可执行文件
install(TARGETS
  serial_ModbusRTU_slavery_node
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

