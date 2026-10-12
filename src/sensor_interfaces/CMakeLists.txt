cmake_minimum_required(VERSION 3.8)
project(sensor_interfaces)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── 1. 找工具 ────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)   # 生成器：.msg → .hpp / .py
find_package(std_msgs REQUIRED)                    # msg 里用了 std_msgs/Header

# ── 2. 声明要生成哪些接口 ─────────────────────────────────────
# 【核心】整个 CMakeLists 里真正不可省的只有这一段。
# 后面的 DEPENDENCIES 写的是「本包 msg 里引用了哪些外部消息类型」——
# 漏了它会报 "Could not find msg 'Header'" 之类的错。
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorData.msg"
  DEPENDENCIES
    std_msgs
)

# ── 3. 把运行时依赖导出给下游包 ──────────────────────────────
# 少了它，依赖 sensor_interfaces 的包在运行时可能找不到消息模块。
ament_export_dependencies(rosidl_default_runtime)

ament_package()
