cmake_minimum_required(VERSION 3.21)
project(CPP3D)

set(CMAKE_CXX_STANDARD 20)

add_executable(CPP3D main.cpp)
#find_package(OpenGL REQUIRED)
find_package(glfw3 3.3 REQUIRED)
find_package(OpenCV REQUIRED )
#find_package(OpenMP)
#if (OPENMP_FOUND)
#    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
#endif()


find_package(realsense2 REQUIRED)
if(WIN32)
    find_package(Open3D HINTS ${CMAKE_INSTALL_PREFIX}/CMake)
else()
    find_package(Open3D HINTS ${CMAKE_INSTALL_PREFIX}/lib/CMake)
    list(APPEND Open3D_LIBRARIES dl)
endif()

include_directories( ${OpenCV_INCLUDE_DIRS} )

link_directories(${Open3D_LIBRARY_DIRS})

target_link_libraries(CPP3D  glfw ${OpenCV_LIBS} ${realsense2_LIBRARY} ${Open3D_LIBRARIES})
target_include_directories(CPP3D PUBLIC ${Open3D_INCLUDE_DIRS})
