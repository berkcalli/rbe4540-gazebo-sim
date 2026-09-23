cmake_minimum_required(VERSION 3.16)
project(pcl_python_merlab LANGUAGES C CXX)
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation search features registration surface io)

pybind11_add_module(pcl_python_merlab MODULE NO_EXTRAS
  src/bindings.cpp src/filters.cpp src/geometry.cpp src/features.cpp
  src/registration.cpp src/surface_io.cpp)
target_compile_features(pcl_python_merlab PRIVATE cxx_std_17)
target_include_directories(pcl_python_merlab SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(pcl_python_merlab PRIVATE ${PCL_LIBRARIES})
add_definitions(${PCL_DEFINITIONS})
ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(TARGETS pcl_python_merlab LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}")
install(FILES API.md DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(pcl_api test/test_api.py
    ENV "PYTHONPATH=$<TARGET_FILE_DIR:pcl_python_merlab>"
    TIMEOUT 60)
endif()
ament_package()
