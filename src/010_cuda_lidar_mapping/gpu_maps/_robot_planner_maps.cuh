#ifndef _ROBOT_PLANNER_MAPS_CUH_
#define _ROBOT_PLANNER_MAPS_CUH_

#include "_gpu_map_ui8.cuh"
#include "../gpu_errchk/gpu_errchk.cuh"
#include "../dh/ht_matrix.hpp"

#include "opencv2/opencv.hpp"

class _RobotPlannerMaps
{
public:
// DEVICE MEMORY
    _GpuMap_UI8 dev_heightmap;
    _GpuMap_UI8 dev_costmap;

    double* dev_dk_matrix;
    float* dev_laser_scan;

    float* dev_debug;
    float* host_debug;

// HOST MEMORY
    cv::Mat host_heightmap;
    cv::Mat host_costmap;

    float map_orient;       // Clock counter-wise angle between East and vector from Start to End points
    float map_scale;        // One meter in real world is equal to this number of fields

    int map_pow2_divider;   // Map size must be divisible by this number
    int map_min_offset;     // Minimum number of fields between Start/Stop points and edge of map
    int map_offset_pix;

    int cmap_refresh_size;  // Size of square, within which costmap is refreshed in single iteration

    int laser_rays;         // Number of lidar's laser rays

    HTMatrix dk_matrix;

public:

    _RobotPlannerMaps();
    ~_RobotPlannerMaps();

    void allocateMaps(float x_deviation_meters, float y_deviation_meters);
    void resizeMaps(float x_deviation_meters, float y_deviation_meters);
    void freeMaps();

    void allocateLaserScan(int laser_rays);
    void freeLaserScan();

    void cudaDebugInfo();


};

#endif
