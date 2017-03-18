cmake_minimum_required(VERSION 2.8.3)
project(cuda_training)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-gencode arch=compute_30,code=sm_30)

find_package(CUDA REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    cv_bridge
    image_transport
)

find_package(OpenCV 3.0.0 REQUIRED)

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
    src/003_cuda_class/main.cpp
    src/003_cuda_class/cuda_class.cu)
target_link_libraries(cuda_class ${catkin_LIBRARIES})

cuda_add_executable(topics_addition_class
    src/004_topics_addition_class/main.cpp
    src/004_topics_addition_class/addition.cu)
target_link_libraries(topics_addition_class ${catkin_LIBRARIES})

cuda_add_executable(errors_tracking
    src/005_errors_tracking/main.cpp
    src/005_errors_tracking/addition.cu
    src/005_errors_tracking/gpu_errchk.cu)
target_link_libraries(errors_tracking ${catkin_LIBRARIES})

cuda_add_executable(path_planner_skelet
    src/006_path_planner_skelet/main.cpp
    src/006_path_planner_skelet/ros_wrapper.cpp
    src/006_path_planner_skelet/gpu_algorithm.cu
    src/006_path_planner_skelet/gpu_errchk.cu)
target_link_libraries(path_planner_skelet ${catkin_LIBRARIES})
cuda_add_executable(path_planner_skelet_pub_hmap
    src/006_path_planner_skelet/pub_hmap/main.cpp)
target_link_libraries(path_planner_skelet_pub_hmap ${catkin_LIBRARIES})

cuda_add_executable(path_planner_segment_cost
    src/007_path_planner_segment_cost/main.cpp
    src/007_path_planner_segment_cost/ros_wrapper.cpp
    src/007_path_planner_segment_cost/gpu_path_planner.cu
    src/007_path_planner_segment_cost/gpu_errchk.cu)
target_link_libraries(path_planner_segment_cost ${catkin_LIBRARIES})
cuda_add_executable(path_planner_segment_cost_pub_hmap
    src/007_path_planner_segment_cost/pub_hmap/main.cpp)
target_link_libraries(path_planner_segment_cost_pub_hmap ${catkin_LIBRARIES})
