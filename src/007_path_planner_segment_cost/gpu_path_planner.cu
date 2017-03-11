#include "gpu_path_planner.cuh"

__global__ void kernelPathPlanning(double *d_odom, double *d_goal, uint8_t *d_hmap, double *d_cost){
    *d_cost = *(d_odom+1) + *(d_goal+2) + *(d_hmap+3);
}

GPUPathPlanner::GPUPathPlanner(){

}

GPUPathPlanner::~GPUPathPlanner(){
    gpuFree();
}

void GPUPathPlanner::gpuSetup(uint32_t hmap_rows, uint32_t hmap_cols){
    this->hmap_rows = hmap_rows;
    this->hmap_cols = hmap_cols;
    gpuErrchk( cudaMalloc((void**)&d_odom, 3*sizeof(double)) );
    gpuErrchk( cudaMalloc((void**)&d_goal, 3*sizeof(double)) );
    gpuErrchk( cudaMalloc((void**)&d_hmap, hmap_rows*hmap_cols*sizeof(uint8_t)) );
    gpuErrchk( cudaMalloc((void**)&d_cost, 1*sizeof(double)) );
}

void GPUPathPlanner::gpuCopyInputToDevice(double *&h_odom, double *&h_goal, uint8_t *&h_hmap){
    gpuErrchk( cudaMemcpy(d_odom, h_odom, 3*sizeof(double), cudaMemcpyHostToDevice) );
    gpuErrchk( cudaMemcpy(d_goal, h_goal, 3*sizeof(double), cudaMemcpyHostToDevice) );
    gpuErrchk( cudaMemcpy(d_hmap, h_hmap, hmap_rows*hmap_cols*sizeof(uint8_t), cudaMemcpyHostToDevice) );
}

void GPUPathPlanner::gpuExecuteKernel(){
    kernelPathPlanning<<<1,1>>>(d_odom, d_goal, d_hmap, d_cost);
    gpuErrchk( cudaPeekAtLastError() );
    gpuErrchk( cudaDeviceSynchronize() );
}


void GPUPathPlanner::gpuCopyOutputToHost(double *&h_cost){
    gpuErrchk( cudaMemcpy(h_cost, d_cost, 1*sizeof(double), cudaMemcpyDeviceToHost) );
}

void GPUPathPlanner::gpuFree(){
    gpuErrchk( cudaFree(d_odom) );
    gpuErrchk( cudaFree(d_goal) );
    gpuErrchk( cudaFree(d_hmap) );
    gpuErrchk( cudaFree(d_cost) );
}
