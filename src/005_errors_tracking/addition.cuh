#ifndef CUDA_ADDITION_CLASS_HPP_
#define CUDA_ADDITION_CLASS_HPP_

#include "gpu_errchk.cuh"

#include <cuda.h>

#include <ros/ros.h>
#include <std_msgs/Int32.h>

class Addition{
    int *d_a, *d_b, *d_c;
    int *h_a, *h_b, *h_c;

    ros::NodeHandle *nh;
    ros::Subscriber sub_a;
    ros::Subscriber sub_b;
    ros::Publisher pub_c;

public:
    Addition(ros::NodeHandle *nh);
    ~Addition();

    void callbackA(const std_msgs::Int32 msg);
    void callbackB(const std_msgs::Int32 msg);

    void allocateDeviceMemory();
    void copyInputToDevice();
    void runKernelGPU();
    void copyOutputToHost();
    void freeDeviceMemeory();

};

#endif
