#ifndef GPU_001_LIDAR_MAPPING_CUH_
#define GPU_001_LIDAR_MAPPING_CUH_

#include "_robot_planner_maps.cuh"
#include "ros/_ros_buffor.hpp"

#include "ht_matrix.hpp"

#include "octave_variable.hpp"

#include "gpu_types.cuh"

#include <stdio.h>
#include <opencv2/opencv.hpp>

HTMatrix dkWorldToLidar( double tx, double ty, double tz, double qx, double qy, double qz, double qw, double th2, const double a1, const double d2, const double al3);

HTMatrixLidarCPU dkWorldToLidarReduced( double tx, double ty, double tz, double qx, double qy, double qz, double qw, double th2, const double a1, const double d2, const double al3);

__device__ inline Point3F32 dkLidarToScan(HTMatrixLidarCPU *dk_cpu, float th5, float a5);

__device__ inline Point2I32 mapRealToGPU(float point_x, float point_y, float map_orient, float map_scale, float map_offset_pix);


class GpuLidarMapping
{
    _RobotPlannerMaps *_rpm;
    _ROSBuffor *_ros;

public:
    GpuLidarMapping(_RobotPlannerMaps *_rpm, _ROSBuffor *_ros);

    void copyInputToDevice();
    void executeKernel();
    void copyOutputToHost();

    void display();
};

#endif
