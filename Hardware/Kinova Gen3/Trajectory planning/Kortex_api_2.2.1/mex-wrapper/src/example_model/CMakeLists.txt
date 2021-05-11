cmake_minimum_required(VERSION 3.5)
project(example_model)
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
add_compile_options(-D_OS_UNIX -D_POSIX_C_SOURCE=199309L -DMODEL=example_model -DNUMST=1 -DNCSTATES=0 -DHAVESTDIO -DMODEL_HAS_DYNAMICALLY_LOADED_SFCNS=0 -DCLASSIC_INTERFACE=0 -DALLOCATIONFCN=0 -DTID01EQ=0 -DTERMFCN=1 -DONESTEPFCN=1 -DMAT_FILE=0 -DMULTI_INSTANCE_CODE=1 -DINTEGER_CODE=0 -DMT=0 -DROS2_PROJECT -DSTACK_SIZE=64 -D__MW_TARGET_USE_HARDWARE_RESOURCES_H__ -DRT=RT )
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package( rclcpp REQUIRED)
find_package( tinyxml2 REQUIRED)
include_directories("/include")
include_directories("D:/IST/ORIENT_repos/Tests/KINOVA Gen3/Trajectory planning/matlab_simplified_api_2.2.1/simplified_api/install/package/lib/Release")
include_directories("../include")
include_directories("include")
include_directories("include/client")
include_directories("include/client_stubs")
include_directories("include/common")
include_directories("include/messages")
include_directories("D:/IST/ORIENT_repos/Tests/KINOVA Gen3/Trajectory planning/matlab_simplified_api_2.2.1/simplified_api/install/package/include")
include_directories("D:/IST/ORIENT_repos/Tests/KINOVA Gen3/Trajectory planning/matlab_simplified_api_2.2.1/simplified_api/kortex_api/cpp/linux_gcc_x86-64/include")
include_directories("lib")
include_directories("include/example_model")
set (CMAKE_SKIP_BUILD_RPATH false)
set (CMAKE_BUILD_WITH_INSTALL_RPATH true)
set (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
link_directories("./lib")
# Node
add_executable(example_model 
src/example_model.cpp
src/example_model_data.cpp
src/main.cpp
src/ros2nodeinterface.cpp
src/slros_busmsg_conversion.cpp
src/kortexApiWrapper.cpp
src/proto_converter.cpp
)
ament_target_dependencies(
  example_model
  "rclcpp"
  "tinyxml2"
)
target_include_directories(example_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(example_model 
	libSimplifiedApi.a
    )
install(TARGETS example_model
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})
ament_export_include_directories(
  include
)
ament_package()
# Generated 13-Apr-2021 16:36:32
# Copyright 2019 The MathWorks, Inc.
