#ifndef GPU_PATH_PLANNER_CUH_
#define GPU_PATH_PLANNER_CUH_

#include <cuda.h>
#include "gpu_errchk.cuh"

class GPUPathPlanner{

    double *d_odom;
    double *d_goal;
    uint8_t *d_hmap;

    double *d_cost;

    uint32_t hmap_rows, hmap_cols;

public:
    GPUPathPlanner();
    ~GPUPathPlanner();

    void gpuSetup(uint32_t hmap_rows, uint32_t hmap_cols);
    void gpuCopyInputToDevice(double *&h_odom, double *&h_goal, uint8_t *&h_hmap);
    void gpuExecuteKernel();
    void gpuCopyOutputToHost(double *&h_cost);
    void gpuFree();
};

#endif
