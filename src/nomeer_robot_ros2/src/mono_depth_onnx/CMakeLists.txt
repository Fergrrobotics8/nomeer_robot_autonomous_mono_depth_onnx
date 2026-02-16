cmake_minimum_required(VERSION 3.8)
project(mono_depth_onnx)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  mono_depth_onnx/image_source_node.py
  mono_depth_onnx/depth_inference_node.py
  mono_depth_onnx/depth_metric_node.py
  mono_depth_onnx/depth_visualizer_node.py
  mono_depth_onnx/autonomous_depth_safety_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config models scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
