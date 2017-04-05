#ifndef GPU_PATH_PLANNER_CUH_
#define GPU_PATH_PLANNER_CUH_

#include <cuda.h>
#include <vector_types.h>
#include "gpu_errchk.cuh"

#define MASK_DIM_ROWS 8
#define MASK_DIM_COLS 8

#define CMAP_OFFSET_ROWS ((MASK_DIM_ROWS - 1)/2)
#define CMAP_OFFSET_COLS ((MASK_DIM_COLS - 1)/2)

#define HMAP_DIM_ROWS 1024
#define HMAP_DIM_COLS 1024

#define CMAP_DIM_ROWS (HMAP_DIM_ROWS - 2*CMAP_OFFSET_ROWS)
#define CMAP_DIM_COLS (HMAP_DIM_COLS - 2*CMAP_OFFSET_COLS)


class GPUPathPlanner{

    double *d_odom;
    double *d_goal;
    double *d_debug;

    uint8_t *d_hmap;
    uint8_t *d_cmap;

    uint32_t hmap_rows, hmap_cols;
    uint32_t cmap_rows, cmap_cols;

    dim3 block_dim;
    dim3 grid_dim;

public:
    GPUPathPlanner();
    ~GPUPathPlanner();

    void gpuSetup(uint32_t hmap_rows, uint32_t hmap_cols);
    void gpuCopyInputToDevice(double *h_odom, double *h_goal, uint8_t *h_hmap);
    void gpuExecuteKernel();
    void gpuCopyOutputToHost(double *h_debug, uint8_t *h_cmap);
    void gpuFree();
};

#endif
