#include "gpu_path_planner.cuh"


__global__ void kernelPathPlanning(double *d_odom, double *d_goal, uint8_t *d_hmap, uint8_t *d_cmap, double *d_debug)
{
    uint32_t idx = blockDim.x * blockIdx.x + threadIdx.x;
    uint32_t idy = blockDim.y * blockIdx.y + threadIdx.y;

    uint32_t cmap_idx = idx + idy * blockDim.x * gridDim.x;
    uint32_t hmap_idx = (CMAP_OFFSET_ROWS + idy)*(blockDim.x*gridDim.x+2*CMAP_OFFSET_COLS) + CMAP_OFFSET_COLS + idx;

    uint32_t hmap_width = blockDim.x*gridDim.x+2*CMAP_OFFSET_COLS;

    // d_cmap[cmap_idx] = d_hmap[hmap_idx];

    float avrg = 0;
    for(int row = -CMAP_OFFSET_ROWS; row < CMAP_OFFSET_ROWS+1; row++)
    {
        for(int col = -CMAP_OFFSET_COLS; col < CMAP_OFFSET_COLS+1; col++)
        {
            avrg += (float) d_hmap[hmap_idx + row*hmap_width + col];
        }
    }
    avrg /= (float) (MASK_DIM_COLS * MASK_DIM_ROWS);


    float variance = 0;
    for(int row = -CMAP_OFFSET_ROWS; row < CMAP_OFFSET_ROWS+1; row++)
    {
        for(int col = -CMAP_OFFSET_COLS; col < CMAP_OFFSET_COLS+1; col++)
        {
            float diff = (float) (avrg - d_hmap[hmap_idx + row*hmap_width + col]);
            variance += diff * diff;
        }
    }
    d_cmap[cmap_idx] = (uint8_t) (variance / 1024);

    d_debug[0] = 999;
}

GPUPathPlanner::GPUPathPlanner()
{

}

GPUPathPlanner::~GPUPathPlanner()
{
    gpuFree();
}

void GPUPathPlanner::gpuSetup(uint32_t hmap_rows, uint32_t hmap_cols)
{
    this->hmap_rows = hmap_rows;
    this->hmap_cols = hmap_cols;

    this->cmap_rows = hmap_rows - 2*CMAP_OFFSET_ROWS;
    this->cmap_cols = hmap_cols - 2*CMAP_OFFSET_COLS;

    gpuErrchk( cudaMalloc((void**)&d_odom, 3*sizeof(double)) );
    gpuErrchk( cudaMalloc((void**)&d_goal, 3*sizeof(double)) );
    gpuErrchk( cudaMalloc((void**)&d_hmap, hmap_rows*hmap_cols*sizeof(uint8_t)) );
    gpuErrchk( cudaMalloc((void**)&d_cmap, cmap_rows*cmap_cols*sizeof(uint8_t)) );
    gpuErrchk( cudaMalloc((void**)&d_debug, 32*sizeof(double)) );
    gpuErrchk( cudaDeviceSynchronize() );

    block_dim = dim3(1, 1);
    grid_dim = dim3(CMAP_DIM_ROWS, CMAP_DIM_COLS);

}

void GPUPathPlanner::gpuCopyInputToDevice(double *h_odom, double *h_goal, uint8_t *h_hmap)
{
    gpuErrchk( cudaMemcpy(d_odom, h_odom, 3*sizeof(double), cudaMemcpyHostToDevice) );
    gpuErrchk( cudaMemcpy(d_goal, h_goal, 3*sizeof(double), cudaMemcpyHostToDevice) );
    gpuErrchk( cudaMemcpy(d_hmap, h_hmap, hmap_rows*hmap_cols*sizeof(uint8_t), cudaMemcpyHostToDevice) );
    gpuErrchk( cudaDeviceSynchronize() );
}

void GPUPathPlanner::gpuExecuteKernel()
{
    kernelPathPlanning<<<grid_dim, block_dim>>>(d_odom, d_goal, d_hmap, d_cmap, d_debug);
    gpuErrchk( cudaPeekAtLastError() );
    gpuErrchk( cudaDeviceSynchronize() );
}


void GPUPathPlanner::gpuCopyOutputToHost(double *h_debug, uint8_t *h_cmap)
{
    gpuErrchk( cudaMemcpy(h_debug, d_debug, 32*sizeof(double), cudaMemcpyDeviceToHost) );
    gpuErrchk( cudaMemcpy(h_cmap, d_cmap, cmap_rows*cmap_cols*sizeof(uint8_t), cudaMemcpyDeviceToHost) );
    gpuErrchk( cudaDeviceSynchronize() );
}

void GPUPathPlanner::gpuFree()
{
    gpuErrchk( cudaFree(d_odom) );
    gpuErrchk( cudaFree(d_goal) );
    gpuErrchk( cudaFree(d_hmap) );
    gpuErrchk( cudaFree(d_cmap) );
    gpuErrchk( cudaFree(d_debug) );
}
