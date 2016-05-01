cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${ROS_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${EIGEN3_INCLUDE_DIR}

)

rosbuild_add_executable(${PROJECT_NAME}
src/main.cpp

src/ORBextractor.cc
src/ORBmatcher.cc
src/Frame.cc
src/MapPoint.cc
src/Map.cc
src/KeyFrameDatabase.cc

src/KeyFrame.cc
src/Converter.cc


src/quickshift.c
src/generic.c
src/host.c


src/ModelFrame.cc
src/ObjectRecognition.cc

src/ModelPoint.cc
src/ModelDataBase.cc
src/random.c
src/Model.cc
)
rosbuild_add_boost_directories()
rosbuild_link_boost(${PROJECT_NAME} thread)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
cholmod
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so

)

