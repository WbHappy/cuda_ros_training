#ifndef GPU_ALGORITHM_CUH_
#define GPU_ALGORITHM_CUH_

#define ODOM_LEN 3
#define GOAL_LEN 3

#define HMAP_ROWS 256
#define HMAP_COLS 256

#include <cuda.h>
#include "gpu_errchk.cuh"

class GPUAlgorithm{

    double *d_odom;
    double *d_goal;
    uint8_t *d_hmap;

    double *h_odom;
    double *h_goal;
    uint8_t *h_hmap;

protected:
    uint8_t *h_hmap_buff;

public:
    GPUAlgorithm();
    ~GPUAlgorithm();

    void setupHost(double *&h_odom, double *&h_goal, uint8_t *&h_hmap);
    void setupCuda();
    void copyInputToDevice();
    void copyInputToDevice(double *&h_odom, double *&h_goal, uint8_t *&h_hmap);
    void executeKernel();
    void copyOutputToHost();
    void copyOutputToHost(double *&h_odom, double *&h_goal, uint8_t *&h_hmap);
    void freeCuda();

};

#endif
