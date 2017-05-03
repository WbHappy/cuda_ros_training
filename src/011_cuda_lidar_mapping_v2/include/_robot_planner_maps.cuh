#ifndef _ROBOT_PLANNER_MAPS_CUH_
#define _ROBOT_PLANNER_MAPS_CUH_

#include "ht_matrix.hpp"

#include "cpu_map_i16.hpp"

#include "gpu_map_i16.cuh"
#include "gpu_types.cuh"
#include "gpu_errchk.cuh"

#include "opencv2/opencv.hpp"

class _RobotPlannerMaps
{
public:
// DEVICE MEMORY
    GpuMapI16 dev_heightmap;
    GpuMapI16 dev_costmap;

    float* dev_dk_matrix;
    float* dev_laser_scan;

    float* dev_debug;


// HOST MEMORY

    CpuMapI16 host_heightmap;
    CpuMapI16 host_costmap;

    HTMatrix dk_matrix;
    HTMatrixLidarCPU dk_cpu;

    float map_orient;       // Clock counter-wise angle between East and vector from Start to End points
    int map_scale;        // One meter in real world is equal to this number in map pixel position (X,Y)
    int height_scale;     // One meter in real world is equal to this number in map pixel value (Z)


    int map_pow2_divider;   // Map size must be divisible by this number
    int map_meters_offset;     // Minimum number of fields between Start/Stop points and edge of map
    int map_offset_pix;

    int cmap_refresh_size;  // Size of square, within which costmap is refreshed in single iteration

    int laser_rays;         // Number of lidar's laser rays


    float* host_debug;

public:

    _RobotPlannerMaps();
    ~_RobotPlannerMaps();

    void allocateConstSizeMemory();
    void allocateMaps(float x_deviation_meters, float y_deviation_meters);
    void allocateLaserScan(int laser_rays);

    void freeConstSizeMemory();
    void freeMaps();
    void freeLaserScan();

    void resizeMaps(float x_deviation_meters, float y_deviation_meters);

    void cudaDebugInfo();
};

#endif
