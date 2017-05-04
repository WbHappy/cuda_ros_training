#ifndef GPU_002_COST_MAPPING_CUH_
#define GPU_002_COST_MAPPING_CUH_

#include "_robot_planner_maps.cuh"
#include "ros/_ros_buffor.hpp"

#include "ht_matrix.hpp"

#include "gpu_types.cuh"
#include "gpu_errchk.cuh"

#include <cuda.h>

class GpuCostMapping
{
    _RobotPlannerMaps *_rpm;
    _ROSBuffor *_ros;


public:
    float cmap_refresh_radius_meters;  // Radius of area, within which costmap is refreshed in single iteration
    int cmap_refresh_radius_pix;

    int cost_mask_radius;   // "Radius" of square making mask for pixel cost calculation (for 31x31 square "Radius" will be 16)
    int unknown_field_cost;   // Cost of empty pixel

    int costmap_borders_value; // Value assigned to borders of costmap - it should be high enought to not allow rover to travell there

public:
    GpuCostMapping(_RobotPlannerMaps *_rpm, _ROSBuffor *_ros);

    void drawInitialCostmapBorders();

    void copyInputToDevice();
    void executeKernel();
    void copyOutputToHost();

    void display();
};

#endif
