cmake_minimum_required(VERSION 2.8.3)
project(cuda_training)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-gencode arch=compute_30,code=sm_30)

find_package(CUDA REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
)

include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package()

cuda_add_executable(hello_world
    src/001_hello_world/main.cu)
target_link_libraries(hello_world ${catkin_LIBRARIES})

cuda_add_executable(topics_addition
    src/002_topics_addition/main.cpp
    src/002_topics_addition/addition.cu)
target_link_libraries(topics_addition ${catkin_LIBRARIES})

cuda_add_executable(cuda_class
    src/002_cuda_class/main.cpp
    src/002_cuda_class/cuda_class.cu)
target_link_libraries(cuda_class ${catkin_LIBRARIES})
