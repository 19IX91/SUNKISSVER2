cmake_minimum_required(VERSION 3.12)
project(r1_controller)

set(CMAKE_CXX_STANDARD 17)
option(R1_TRACKING_OFFLINE_ONLY "Build the tracker without SDK/DDS or motor access" OFF)
set(UNITREE_SDK2_ROOT "" CACHE PATH "Optional uninstalled Unitree SDK2 checkout")

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)
find_package(ZLIB REQUIRED)
find_path(R1_EIGEN_INCLUDE_DIR Eigen/Dense
  HINTS ${CMAKE_PREFIX_PATH}
  PATH_SUFFIXES include/eigen3
  REQUIRED)

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    set(ONNXRUNTIME_DIR "${PROJECT_SOURCE_DIR}/../../thirdparty/onnxruntime-linux-x64-1.22.0")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
    set(ONNXRUNTIME_DIR "${PROJECT_SOURCE_DIR}/../../thirdparty/onnxruntime-linux-aarch64-1.22.0")
endif()

add_executable(r1_tracking_offline src/tracking_offline.cpp)
target_include_directories(r1_tracking_offline PRIVATE include ${R1_EIGEN_INCLUDE_DIR} ${ONNXRUNTIME_DIR}/include)
target_link_libraries(r1_tracking_offline PRIVATE yaml-cpp ${ONNXRUNTIME_DIR}/lib/libonnxruntime.so.1.22.0)
set_target_properties(r1_tracking_offline PROPERTIES BUILD_RPATH "${ONNXRUNTIME_DIR}/lib")
include(CTest)
if(BUILD_TESTING)
    add_executable(r1_tracking_contract_test tests/tracking_contract.cpp)
    target_include_directories(r1_tracking_contract_test PRIVATE include ${R1_EIGEN_INCLUDE_DIR} ${ONNXRUNTIME_DIR}/include)
    target_link_libraries(r1_tracking_contract_test PRIVATE yaml-cpp ${ONNXRUNTIME_DIR}/lib/libonnxruntime.so.1.22.0)
    add_test(NAME r1_tracking_estimator_contract COMMAND r1_tracking_contract_test)
endif()
if(R1_TRACKING_OFFLINE_ONLY)
    return()
endif()

if(UNITREE_SDK2_ROOT)
    include_directories(${UNITREE_SDK2_ROOT}/include ${UNITREE_SDK2_ROOT}/thirdparty/include ${UNITREE_SDK2_ROOT}/thirdparty/include/ddscxx)
    link_directories(${UNITREE_SDK2_ROOT}/lib/${CMAKE_SYSTEM_PROCESSOR} ${UNITREE_SDK2_ROOT}/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR})
endif()

include_directories(
  ${R1_EIGEN_INCLUDE_DIR}
  /usr/local/include/ddscxx
  /usr/local/include/iceoryx/v2.0.2
  include
  ${ONNXRUNTIME_DIR}/include
  ${PROJECT_SOURCE_DIR}/../../include/
  ${PROJECT_SOURCE_DIR}/../../thirdparty/cnpy
)

file(GLOB CNPY_SRC ${PROJECT_SOURCE_DIR}/../../thirdparty/cnpy/*.cpp)

add_library(cnpy_lib STATIC ${CNPY_SRC})

link_libraries(
  unitree_sdk2
  ddsc
  ddscxx
  rt
  pthread
  Boost::program_options
  yaml-cpp
  fmt
  ${ONNXRUNTIME_DIR}/lib/libonnxruntime.so.1.22.0
  cnpy_lib
  ZLIB::ZLIB
)

file(GLOB_RECURSE ADD_SRC_LIST
  ${PROJECT_SOURCE_DIR}/src/*.cpp
)
list(FILTER ADD_SRC_LIST EXCLUDE REGEX "/tracking_offline.cpp$")

add_library(${PROJECT_NAME}_lib ${ADD_SRC_LIST})
link_libraries(${PROJECT_NAME}_lib)

add_executable(r1_ctrl main.cpp)
set_target_properties(r1_ctrl PROPERTIES BUILD_RPATH "${ONNXRUNTIME_DIR}/lib;${UNITREE_SDK2_ROOT}/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}")
