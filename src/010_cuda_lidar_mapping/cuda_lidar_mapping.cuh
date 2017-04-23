#ifndef CUDA_LIDAR_MAPPING_CUH_
#define CUDA_LIDAR_MAPPING_CUH_

#include "gpu_maps/_robot_planner_maps.hpp"

#include <stdio.h>

class CudaLidarMapping
{
    _RobotPlannerMaps *_rpm;

public:
    CudaLidarMapping(_RobotPlannerMaps *_rpm);

    void copyInputToDevice();
    void executeKernel();
    void copuOutputToHost();

    void printAqq(){
        printf("AQQ!\n");
    }

};

#endif
