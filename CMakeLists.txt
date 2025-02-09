cmake_minimum_required(VERSION 3.8)  # 设置 CMake 的最小版本要求  
project(point_lio)                     # 定义项目名称为 point_lio  

# 设置构建类型为 Release，如果没有指定构建类型  
if(NOT CMAKE_BUILD_TYPE)  
  set(CMAKE_BUILD_TYPE Release)  
endif()  

# 添加 C++14 的编译选项  
ADD_COMPILE_OPTIONS(-std=c++14)  
ADD_COMPILE_OPTIONS(-std=c++14)  # 重复添加，建议去掉重复  
set(CMAKE_CXX_FLAGS "-std=c++14 -O3")  # 设置 C++ 编译标志为 C++14，并开启优化  

# 定义根目录的宏  
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")  

# 设置 C 语言的编译选项  
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")  # 开启异常处理  

# 设置 C++ 标准和相关选项  
set(CMAKE_CXX_STANDARD 14)  
set(CMAKE_CXX_STANDARD_REQUIRED ON)  # 要求必须使用 C++14  
set(CMAKE_CXX_EXTENSIONS OFF)        # 禁用编译器扩展  
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -std=c++0x -std=c++14 -fexceptions")  # 添加线程支持和异常处理  
set(CMAKE_POSITION_INDEPENDENT_CODE ON)  # 生成位置无关代码，适用于共享库  

# 检查 CPU 架构，以便进行多线程处理  
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")  
  include(ProcessorCount)  # 包含处理器计数模块  
  ProcessorCount(N)        # 计算可用的处理器数量  
  message(STATUS "Processor number:  ${N}")  # 输出可用处理器数量  

  # 根据处理器数量设置多线程处理的相关定义  
  if(N GREATER 5)  
    add_definitions(-DMP_EN)  # 启用多处理支持  
    add_definitions(-DMP_PROC_NUM=4)  # 设置处理器数量为 4  
    message(STATUS "core for MP:  3")  
  elseif(N GREATER 3)  
    math(EXPR PROC_NUM "${N} - 2")  # 计算处理器数量，减去 2  
    add_definitions(-DMP_EN)  # 启用多处理支持  
    add_definitions(-DMP_PROC_NUM="${PROC_NUM}")  # 设置处理器数量  
    message(STATUS "core for MP:  ${PROC_NUM}")  
  else()  
    add_definitions(-DMP_PROC_NUM=1)  # 如果处理器数量少于 4，设置为 1  
  endif()  
else()  
  add_definitions(-DMP_PROC_NUM=1)  # 非 x86 架构，设置为 1  
endif()  

# 查找所需的包  
find_package(ament_cmake REQUIRED)  # 查找 ament_cmake 包  
find_package(rclcpp REQUIRED)        # 查找 rclcpp 包  
find_package(rclpy REQUIRED)         # 查找 rclpy 包  
find_package(geometry_msgs REQUIRED) # 查找 geometry_msgs 包  
find_package(nav_msgs REQUIRED)      # 查找 nav_msgs 包  
find_package(sensor_msgs REQUIRED)   # 查找 sensor_msgs 包  
find_package(visualization_msgs REQUIRED) # 查找 visualization_msgs 包  
find_package(pcl_ros REQUIRED)       # 查找 pcl_ros 包  
find_package(pcl_conversions REQUIRED) # 查找 pcl_conversions 包  
find_package(tf2_ros REQUIRED)       # 查找 tf2_ros 包  
find_package(livox_ros_driver2 REQUIRED) # 查找 livox_ros_driver2 包  
find_package(Eigen3 REQUIRED)        # 查找 Eigen3 包  
find_package(OpenMP QUIET)           # 查找 OpenMP 包  
find_package(PythonLibs REQUIRED)    # 查找 Python 库  

# 查找 matplotlibcpp 头文件的路径  
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")  

# 如果找到了 OpenMP，添加相关编译选项  
if(OPENMP_FOUND)  
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")  # 添加 OpenMP 的 C++ 编译标志  
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")        # 添加 OpenMP 的 C 编译标志  
endif()  

# 包含目录  
include_directories(  
  include  
  ${EIGEN3_INCLUDE_DIR}  
  ${PCL_INCLUDE_DIRS}  
  ${PYTHON_INCLUDE_DIRS}  
)  

# 链接目录  
link_directories(  
  include  
  ${PCL_LIBRARY_DIRS}  
)  

# 声明一个 ROS2 可执行文件  
add_executable(pointlio_mapping  
  src/laserMapping.cpp  
  src/li_initialization.cpp  
  src/parameters.cpp  
  src/preprocess.cpp  
  src/Estimator.cpp  
  src/IMU_Processing.cpp  
)  

# 添加 ROS2 依赖  
ament_target_dependencies(pointlio_mapping  
  rclcpp  
  rclpy  
  geometry_msgs  
  nav_msgs  
  sensor_msgs  
  visualization_msgs  
  pcl_ros  
  pcl_conversions  
  tf2_ros  
  livox_ros_driver2  
)  

# 链接库  
target_link_libraries(pointlio_mapping ${PCL_LIBRARIES} ${PYTHON_LIBRARIES})  
target_include_directories(pointlio_mapping PRIVATE ${PYTHON_INCLUDE_DIRS})  

# 安装可执行文件  
install(TARGETS  
  pointlio_mapping  
  DESTINATION lib/${PROJECT_NAME}  
)  

# 安装配置、启动和 RViz 配置文件  
install(  
  DIRECTORY config launch rviz_cfg  
  DESTINATION share/${PROJECT_NAME}  
)  

# 导出依赖项  
ament_export_dependencies(  
  rclcpp  
  rclpy  
  geometry_msgs  
  nav_msgs  
  sensor_msgs  
  pcl_ros  
  pcl_conversions  
  tf2_ros  
  livox_ros_driver2  
  Eigen3  
  PCL  
)  

ament_package()  # 声明一个 ament 包


