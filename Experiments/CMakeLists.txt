cmake_minimum_required(VERSION 2.8)
project(ExperimentCartWalkGradientOptim)

#Rhoban dependencies repositories paths for headers inclusion
set(RHOBAN_MATH_DIRECTORY
    "/home/rhoban/Math/" 
    CACHE STRING "RhobanMath directory")
set(RHOBAN_COMMON_DIRECTORY
    "/home/rhoban/Common/" 
    CACHE STRING "RhobanCommon directory")
set(RHOBAN_UTILS_DIRECTORY
    "/home/rhoban/Utils/" 
    CACHE STRING "RhobanUtils directory")

#Rhoban C++ SDK directory path
set(RHOBAN_SDK_DIRECTORY 
    "/home/rhoban/Edu/sdk/cpp" 
    CACHE STRING "RhobanSdk cpp directory")

#Path to a SDK build directory (with SDK, Utils, Common and library binaries)
set(SDK_BUILD_DIRECTORY 
    "/home/rhoban/Edu/build" 
    CACHE STRING "RhobanSdkBuild directory")

#Thesis Code directory
set(LEPH_CODE_DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    CACHE STRING "LephCode directory")

#Eigen linear algebra library directory
set(EIGEN_DIRECTORY
    "/home/rhoban/Eigen"
    CACHE STRING "Eigen library directory")

#RBDL library directory
set(RBDL_DIRECTORY
    "/home/rhoban/RBDL"
    CACHE STRING "RBDL library directory")

#ZMQ communication library directory
set(ZMQ_DIRECTORY
    ${RHOBAN_UTILS_DIRECTORY}/zmq3.2
    CACHE STRING "ZMQ library directory")

#Enable C++17
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
   
# Header includes
include_directories(
    ${RHOBAN_MATH_DIRECTORY}
    ${RHOBAN_COMMON_DIRECTORY}
    ${RHOBAN_UTILS_DIRECTORY}
    ${RHOBAN_SDK_DIRECTORY}
    ${RHOBAN_SDK_DIRECTORY}/rhoban
    ${SDK_BUILD_DIRECTORY}/common/
    ${LEPH_CODE_DIRECTORY}
    ${LEPH_CODE_DIRECTORY}/Experiments/Utils
    ${EIGEN_DIRECTORY}
    ${ZMQ_DIRECTORY}/include
    ${RBDL_DIRECTORY}/include
    ${RBDL_DIRECTORY}/build/include
    ${RBDL_DIRECTORY}/addons
)

link_directories(
    ${SDK_BUILD_DIRECTORY}
    ${SDK_BUILD_DIRECTORY}/common
    ${SDK_BUILD_DIRECTORY}/utils
    ${SDK_BUILD_DIRECTORY}/utils/yaml-cpp
    ${SDK_BUILD_DIRECTORY}/utils/xml/TinyXml
    ${SDK_BUILD_DIRECTORY}/utils/zmq3.2
    ${RBDL_DIRECTORY}/build
    ${RBDL_DIRECTORY}/build/addons/urdfreader
)

#Build Leph Code
add_subdirectory(${LEPH_CODE_DIRECTORY} ${CMAKE_CURRENT_BINARY_DIR}/LephCode)

#
# TinyXml in RhobanUtils.a have to be
# compiled with -DTIXML_USE_STL to be compatible with
# libtinyxml in the system used by RBDL urdf parse (urdfdom)
#

#Lists of linked library
set(LINKED_LIBRARIES
    LephCode
    RhobanSDKMain 
    RhobanCommon 
    RhobanUtils
    pthread
    yaml-cpp
    TinyXml
    ZeroMQ
    ncurses
)

#Build Experiments
add_executable(ExperimentCartWalkGradientOptim 
    CartWalkGradientOptim/main.cpp
    Utils/MotionCapture.cpp
    Utils/SDKConnection.cpp
)
target_link_libraries(ExperimentCartWalkGradientOptim ${LINKED_LIBRARIES})
add_executable(CartWalkTunner
    CartWalkTunner/main.cpp
    Utils/SDKConnection.cpp
)
target_link_libraries(CartWalkTunner ${LINKED_LIBRARIES})
add_executable(StaticWalkTest
    StaticWalkTest/main.cpp
    Utils/SDKConnection.cpp
)
target_link_libraries(StaticWalkTest ${LINKED_LIBRARIES})
add_executable(SplinesPlayer
    SplinesPlayer/main.cpp
    Utils/SDKConnection.cpp
)
target_link_libraries(SplinesPlayer ${LINKED_LIBRARIES})

