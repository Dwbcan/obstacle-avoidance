# WINDOWS

cmake_minimum_required(VERSION 3.0.0)
project(obstacle-avoidance VERSION 0.1.0)

include(CTest)
enable_testing()

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

add_executable(obstacle-avoidance detection.cpp)


set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(obstacle-avoidance ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT})





# LINUX (UNCOMMENT EVERYTHING BELOW AND COMMENT EVERYTHING ABOVE IF USING LINUX OS)

# cmake_minimum_required(VERSION 3.0.0)
# project( obstacle-avoidance )
# find_package( OpenCV REQUIRED )
# include_directories( ${OpenCV_INCLUDE_DIRS} )
# add_executable( obstacle-avoidance detection.cpp )
# target_link_libraries( obstacle-avoidance ${OpenCV_LIBS} )