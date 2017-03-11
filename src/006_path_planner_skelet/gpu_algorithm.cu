#include "gpu_algorithm.cuh"

__global__ void test_kernel1(double *d_odom, double *d_goal, uint8_t *d_hmap){
    uint32_t tid = blockIdx.x * blockDim.x + threadIdx.x;

    d_hmap[tid] += (int8_t) (d_odom[0] + d_odom[1] + d_odom[2] - d_goal[0] - d_goal[1] - d_goal[2]);
    // d_hmap[tid] += 55;

}

GPUAlgorithm::GPUAlgorithm(){
}

GPUAlgorithm::~GPUAlgorithm(){
    freeCuda();
}

void GPUAlgorithm::setupHost(double *&h_odom, double *&h_goal, uint8_t *&h_hmap){
    this->h_odom = h_odom;
    this->h_goal = h_goal;
    this->h_hmap = h_hmap;

    this->h_hmap_buff = (uint8_t *)malloc(HMAP_ROWS*HMAP_COLS*sizeof(uint8_t));
    printf ("+++ %d +++ \n", h_hmap[12]);
}

void GPUAlgorithm::setupCuda(){
    gpuErrchk( cudaMalloc((void**) &d_odom, ODOM_LEN*sizeof(double)) );
    gpuErrchk( cudaMalloc((void**) &d_goal, GOAL_LEN*sizeof(double)) );
    gpuErrchk( cudaMalloc((void**) &d_hmap, HMAP_ROWS*HMAP_COLS*sizeof(uint8_t)) );
}

void GPUAlgorithm::copyInputToDevice(){
    gpuErrchk( cudaMemcpy(d_odom, h_odom, ODOM_LEN*sizeof(double), cudaMemcpyHostToDevice) );
    gpuErrchk( cudaMemcpy(d_goal, h_goal, GOAL_LEN*sizeof(double), cudaMemcpyHostToDevice) );
    gpuErrchk( cudaMemcpy(d_hmap, h_hmap, HMAP_ROWS*HMAP_COLS*sizeof(uint8_t), cudaMemcpyHostToDevice) );
    gpuErrchk( cudaDeviceSynchronize() );

}

void GPUAlgorithm::copyInputToDevice(double *&h_odom, double *&h_goal, uint8_t *&h_hmap){
    gpuErrchk( cudaMemcpy(d_odom, h_odom, ODOM_LEN*sizeof(double), cudaMemcpyHostToDevice) );
    gpuErrchk( cudaMemcpy(d_goal, h_goal, GOAL_LEN*sizeof(double), cudaMemcpyHostToDevice) );
    gpuErrchk( cudaMemcpy(d_hmap, h_hmap, HMAP_ROWS*HMAP_COLS*sizeof(uint8_t), cudaMemcpyHostToDevice) );
    gpuErrchk( cudaDeviceSynchronize() );

}

void GPUAlgorithm::executeKernel(){
    test_kernel1<<<HMAP_ROWS*HMAP_COLS/256, 256>>>(d_odom, d_goal, d_hmap);
    gpuErrchk( cudaPeekAtLastError() );
    gpuErrchk( cudaDeviceSynchronize() );
}

void GPUAlgorithm::copyOutputToHost(){
    gpuErrchk( cudaMemcpy(h_hmap_buff, d_hmap, HMAP_ROWS*HMAP_COLS*sizeof(uint8_t), cudaMemcpyDeviceToHost) );
    gpuErrchk( cudaDeviceSynchronize() );
}


void GPUAlgorithm::copyOutputToHost(double *&h_odom, double *&h_goal, uint8_t *&h_hmap){
    gpuErrchk( cudaMemcpy(h_hmap_buff, d_hmap, HMAP_ROWS*HMAP_COLS*sizeof(uint8_t), cudaMemcpyDeviceToHost) );
    gpuErrchk( cudaDeviceSynchronize() );

}


void GPUAlgorithm::freeCuda(){
    gpuErrchk( cudaFree(d_odom) );
    gpuErrchk( cudaFree(d_goal) );
    gpuErrchk( cudaFree(d_hmap) );
}
