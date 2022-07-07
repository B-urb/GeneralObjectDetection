cmake_minimum_required(VERSION 3.21)

set(CMAKE_CXX_STANDARD 20)




set(TARGET_NAME CPP3D)
project(${TARGET_NAME})
find_package(realsense2 REQUIRED)
if(WIN32)
    find_package(Open3D HINTS ${CMAKE_INSTALL_PREFIX}/CMake)
else()
    find_package(Open3D HINTS ${CMAKE_INSTALL_PREFIX}/lib/CMake)
    list(APPEND Open3D_LIBRARIES dl)
endif()


find_package(OpenCV REQUIRED )
find_package(depthai CONFIG REQUIRED)

#find_package(depthai CONFIG REQUIRED)
#find_package(OpenGL REQUIRED)
find_package(glfw3 3.3 REQUIRED)
find_package(OpenCV REQUIRED )
#find_package(OpenMP)
#if (OPENMP_FOUND)
#    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
#endif()


include_directories( ${OpenCV_INCLUDE_DIRS} )

link_directories(${Open3D_LIBRARY_DIRS})


#set_property(TARGET ${TARGET_NAME} PROPERTY CXX_STANDARD_REQUIRED ON)
#set_property(TARGET ${TARGET_NAME} PROPERTY CXX_EXTENSIONS OFF)



add_executable(${TARGET_NAME} main.cpp src/utility.cpp src/utility.hpp src/Processing.cpp src/Processing.h src/RealSenseCamera.cpp src/RealSenseCamera.h src/DepthAiCamera.cpp src/DepthAiCamera.h)

target_link_libraries(${TARGET_NAME} depthai::opencv)

target_link_libraries(${TARGET_NAME} glfw ${OpenCV_LIBS} ${realsense2_LIBRARY} ${Open3D_LIBRARIES})
target_include_directories(${TARGET_NAME} PUBLIC ${Open3D_INCLUDE_DIRS})


