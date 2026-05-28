cmake_minimum_required(VERSION 3.16)
project(robot_model)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(urdfdom REQUIRED)
find_package(tinyxml2 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Development.Module)
find_package(pybind11 REQUIRED)

ament_export_dependencies(pinocchio urdfdom tinyxml2)

# Create an empty library for now so compilation succeeds while files are added
file(GLOB_RECURSE SRC_FILES src/*.cpp)
if(NOT SRC_FILES)
  # dummy placeholder
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/dummy.cpp "void dummy(){}")
  list(APPEND SRC_FILES ${CMAKE_CURRENT_BINARY_DIR}/dummy.cpp)
endif()

add_library(${PROJECT_NAME} SHARED
  ${SRC_FILES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  pinocchio::pinocchio
  urdfdom::urdfdom_model
  tinyxml2
)

ament_export_targets(${PROJECT_NAME}Export HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Export
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Python bindings
pybind11_add_module(robot_model_py MODULE
  bindings/python/robot_model.cpp
)

# Ensure the compiled file is still named 'robot_model' for Python imports
set_target_properties(robot_model_py PROPERTIES OUTPUT_NAME robot_model)

target_link_libraries(robot_model_py PRIVATE
  ${PROJECT_NAME}
  Eigen3::Eigen
)

# Install Python module
set(PYTHON_INSTALL_DIR "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")
install(TARGETS robot_model_py
  DESTINATION ${PYTHON_INSTALL_DIR}/robot_model
)

# Install __init__.py
install(FILES bindings/python/__init__.py
  DESTINATION ${PYTHON_INSTALL_DIR}/robot_model
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
