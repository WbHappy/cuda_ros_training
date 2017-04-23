#include "gpu_path_planner.cuh"

__global__ void kernelPathPlanner(double *d_odom, double *d_goal, uint8_t *d_hmap, uint8_t *d_cmap, double *d_debug, uint32_t hmap_width, uint32_t cmap_width)
{
    uint32_t idx = blockDim.x * blockIdx.x + threadIdx.x;
    uint32_t idy = blockDim.y * blockIdx.y + threadIdx.y;

    uint32_t cmap_idx = d_odom[0] + idx - LASER_RANGE;
    uint32_t cmap_idy = d_odom[1] + idy - LASER_RANGE;
    uint32_t cmap_tid = cmap_idx + cmap_idy * cmap_width;

    uint32_t hmap_idx = d_odom[0] + idx - LASER_RANGE + MASK_OFFSET;
    uint32_t hmap_idy = d_odom[1] + idy - LASER_RANGE + MASK_OFFSET;
    uint32_t hmap_tid = hmap_idx + hmap_idy * hmap_width;


    // uint32_t hmap_idx = (MASK_OFFSET + idy)*(blockDim.x*gridDim.x+2*MASK_OFFSET) + MASK_OFFSET + idx;

    int32_t y_min = - MASK_OFFSET;
    int32_t y_max = + MASK_OFFSET + 1;
    int32_t x_min = - MASK_OFFSET;
    int32_t x_max = + MASK_OFFSET + 1;

    // d_cmap[cmap_idx] = d_hmap[hmap_idx];

    float avrg = 0;
    for(int y = y_min; y < y_max; y++)
    {
        for(int x = x_min; x < x_max; x++)
        {
            avrg += (float) d_hmap[hmap_tid + y*hmap_width + x];
        }
    }
    avrg /= (float) (MASK_SIZE * MASK_SIZE);


    float variance = 0;
    for(int y = y_min; y < y_max; y++)
    {
        for(int x = x_min; x < x_max; x++)
        {
            float diff = (float) (avrg - d_hmap[hmap_tid + y*hmap_width + x]);
            variance += diff * diff;
        }
    }
    d_cmap[cmap_tid] = (uint8_t) (variance / 1024);

    d_debug[0] = d_odom[0];
}

__global__ void kernelPathPlanner_SharedMemory(double *d_odom, double *d_goal, uint8_t *d_hmap, uint8_t *d_cmap, double *d_debug, uint32_t hmap_width, uint32_t cmap_width)
{
    uint32_t idx = blockDim.x * blockIdx.x + threadIdx.x;
    uint32_t idy = blockDim.y * blockIdx.y + threadIdx.y;

    uint32_t cmap_idx = d_odom[0] + idx - LASER_RANGE;
    uint32_t cmap_idy = d_odom[1] + idy - LASER_RANGE;
    uint32_t cmap_tid = cmap_idx + cmap_idy * cmap_width;

    uint32_t hmap_idx = d_odom[0] + idx - LASER_RANGE + MASK_OFFSET;
    uint32_t hmap_idy = d_odom[1] + idy - LASER_RANGE + MASK_OFFSET;
    uint32_t hmap_tid = hmap_idx + hmap_idy * hmap_width;

    int32_t y_min = - MASK_OFFSET;
    int32_t y_max = + MASK_OFFSET + 1;
    int32_t x_min = - MASK_OFFSET;
    int32_t x_max = + MASK_OFFSET + 1;

    // Calculating average value for lmap
    float avrg = 0;
    for(int y = y_min; y < y_max; y++)
    {
        for(int x = x_min; x < x_max; x++)
        {
            avrg += (float) d_hmap[hmap_tid + y*hmap_width + x];
        }
    }
    avrg /= (float) (MASK_SIZE * MASK_SIZE);


    // Calculating varianve of lmap
    float variance = 0;
    for(int y = y_min; y < y_max; y++)
    {
        for(int x = x_min; x < x_max; x++)
        {
            float diff = (float) (avrg - d_hmap[hmap_tid + y*hmap_width + x]);
            variance += diff * diff;
        }
    }
    d_cmap[cmap_tid] = (uint8_t) (variance / 1024);

    d_debug[0] = d_odom[0];
}

__global__ void kernelClearMemory(uint8_t *d_cmap)
{
    uint32_t idx = blockDim.x * blockIdx.x + threadIdx.x;
    uint32_t idy = blockDim.y * blockIdx.y + threadIdx.y;
    uint32_t tid = idx + idy * blockDim.x * gridDim.x;

    d_cmap[tid] = 0;
}

GPUPathPlanner::GPUPathPlanner(uint32_t mask_size ,uint32_t laser_range)
{
    this->mask_size = mask_size;
    this->mask_offset = (mask_size -1) / 2;
    this->laser_range = laser_range;

    if(mask_size % 2 != 1)
    {
        printf("ERROR -> WRONG MASK DIMMENSIONS");
        exit(-1);
    }
}

void GPUPathPlanner::gpuSetup(uint32_t hmap_x, uint32_t hmap_y)
{
    this->hmap_x = hmap_x;
    this->hmap_y = hmap_y;
    this->cmap_x = hmap_x - 2*mask_offset;
    this->cmap_y = hmap_y - 2*mask_offset;

    if(cmap_x % 32 != 0 || cmap_y % 32 != 0)
    {
        printf("ERROR -> WRONG MAP DIMMENSIONS");
        exit(-1);
    }
    block_dim = dim3(32,32,1);
    grid_dim = dim3(2*laser_range/32, 2*laser_range/32, 1);

    gpuErrchk( cudaMalloc((void**)&d_odom, 3*sizeof(double)) );
    gpuErrchk( cudaMalloc((void**)&d_goal, 3*sizeof(double)) );
    gpuErrchk( cudaMalloc((void**)&d_debug, 32*sizeof(double)) );

    gpuErrchk( cudaMalloc((void**)&d_hmap, hmap_x * hmap_y * sizeof(uint8_t)) );
    gpuErrchk( cudaMalloc((void**)&d_cmap, cmap_x * cmap_y * sizeof(uint8_t)) );

}


void GPUPathPlanner::gpuCopyHMapToDevice(uint8_t *h_hmap)
{
    gpuErrchk( cudaMemcpy(d_hmap, h_hmap, hmap_x * hmap_y * sizeof(uint8_t), cudaMemcpyHostToDevice) );
}

void GPUPathPlanner::gpuCopyOdomToDevice(double *h_odom)
{
    gpuErrchk( cudaMemcpy(d_odom, h_odom, 3 * sizeof(double), cudaMemcpyHostToDevice) );
}

void GPUPathPlanner::gpuCopyGoalToDevice(double *h_goal)
{
    gpuErrchk( cudaMemcpy(d_goal, h_goal, 3 * sizeof(double), cudaMemcpyHostToDevice) );
}



void GPUPathPlanner::gpuCopyCMapFromDevice(uint8_t *h_cmap)
{
    gpuErrchk( cudaMemcpy(h_cmap, d_cmap, cmap_x * cmap_y * sizeof(uint8_t), cudaMemcpyDeviceToHost) );
}
void GPUPathPlanner::gpuCopyDebugFromDevice(double *h_debug)
{
    gpuErrchk( cudaMemcpy(h_debug, d_debug, 32*sizeof(double), cudaMemcpyDeviceToHost) );
}
void GPUPathPlanner::gpuCopyDebugFromDevice(double *h_debug, uint32_t n_bytes)
{
    gpuErrchk( cudaMemcpy(h_debug, d_debug, n_bytes*sizeof(double), cudaMemcpyDeviceToHost) );
}


void GPUPathPlanner::gpuClearMemory()
{
    kernelClearMemory <<<cmap_x, cmap_y>>> (d_cmap);
    gpuErrchk( cudaPeekAtLastError() );
    gpuErrchk( cudaDeviceSynchronize() );
}

void GPUPathPlanner::gpuExecuteKernel()
{
    kernelPathPlanner <<<grid_dim, block_dim>>> (d_odom, d_goal, d_hmap, d_cmap, d_debug, hmap_y, cmap_y);
    gpuErrchk( cudaPeekAtLastError() );
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
