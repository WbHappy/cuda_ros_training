#ifndef GPU_002_COST_MAPPING_CUH_
#define GPU_002_COST_MAPPING_CUH_

class GpuCostMapping
{
    _RobotPlannerMaps *_rpm;
    _ROSBuffor *_ros;

    int cmap_refresh_radius_meters;  // Radius of area, within which costmap is refreshed in single iteration


public:
    GpuCostMapping(_RobotPlannerMaps *_rpm, _ROSBuffor *_ros);

    void copyInputToDevice();
    void executeKernel();
    void copyOutputToHost();

    void display();
};

#endif
