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

    float* dev_dk_matrix;
    float* dev_laser_scan;

    HTMatrix dk_matrix;
    HTMatrixLidarCPU dk_cpu;

public:
    int laser_rays;         // Number of lidar's laser rays

    float angle_min;
    float angle_max;

    float dk_a1;    // Offset from rover center to LiDAR tower Z-axis
    float dk_d2;    // Height from rover center to LiDAR scanner
    float dk_al3;   // Angle of LiDAR tilt in its Y-axis

    // needed on the begin, to estimate height of area under rover, which cannot be mapped without moving
    int init_circle_height;     // Initial height of circle - should be equal to height from rover center to bottom of wheel (negative number)
    float init_circle_radius;   // Radius of start circle within which pixels are set to init_circle_height value

public:
    GpuLidarMapping(_RobotPlannerMaps *_rpm, _ROSBuffor *_ros);

    void allocateMemory(int laser_rays, float angle_min, float angle_max);
    void freeMemory();

    void drawInitialHeightmapCircle();

    void copyInputToDevice();
    void executeKernel();
    void copyOutputToHost();

    void display();
};

#endif
