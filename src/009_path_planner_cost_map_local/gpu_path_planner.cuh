#ifndef GPU_PATH_PLANNER_CUH_
#define GPU_PATH_PLANNER_CUH_

#include "gpu_errchk.cuh"

#include <cuda.h>
#include <vector_types.h>

#define MASK_SIZE 15
#define MASK_OFFSET ((MASK_SIZE-1)/2)
#define LASER_RANGE 128

class GPUPathPlanner
{
    double *d_odom;
    double *d_goal;
    double *d_debug;

    uint8_t *d_hmap;
    uint8_t *d_cmap;

    uint32_t hmap_x, hmap_y;
    uint32_t cmap_x, cmap_y;

    dim3 block_dim;
    dim3 grid_dim;

    uint32_t mask_size;
    uint32_t mask_offset;
    uint32_t laser_range;

public:

    GPUPathPlanner(uint32_t mask_size ,uint32_t laser_range);

    void gpuSetup(uint32_t hmap_x, uint32_t hmap_y);

    void gpuCopyHMapToDevice(uint8_t *h_hmap);
    void gpuCopyOdomToDevice(double *h_odom);
    void gpuCopyGoalToDevice(double *h_goal);

    void gpuCopyCMapFromDevice(uint8_t *h_cmap);
    void gpuCopyDebugFromDevice(double *h_debug);
    void gpuCopyDebugFromDevice(double *h_debug, uint32_t n_bytes);

    void gpuClearMemory();
    void gpuExecuteKernel();

    void gpuFree();
};

#endif
