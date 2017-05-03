#ifndef CUDA_LIDAR_MAPPING_CUH_
#define CUDA_LIDAR_MAPPING_CUH_

#include "gpu_maps/_robot_planner_maps.cuh"
#include "ros/_ros_buffor.hpp"

#include <stdio.h>
#include <opencv2/opencv.hpp>

#define SCAN_ANGLE 4.7123889803846898576939650749192543262957540990626587314624169 // 270 degrees

class CudaLidarMapping
{
    _RobotPlannerMaps *_rpm;
    _ROSBuffor *_ros;

public:
    CudaLidarMapping(_RobotPlannerMaps *_rpm, _ROSBuffor *_ros);

    void copyInputToDevice();
    void executeKernel();
    void copyOutputToHost();

    void display();

};

#endif
