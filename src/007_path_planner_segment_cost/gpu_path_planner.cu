#include "gpu_path_planner.cuh"

#define MAX_SUBEP_LEN 16

__global__ void kernelPathPlanning(double *d_odom, double *d_goal, uint8_t *d_hmap, double *d_cost){
    // int tid = blockIdx.x*blockDim.x + threadIdx.x;

    // Divide episode to subepisodes
    double ep_len = sqrt((d_odom[0] - d_goal[0])*(d_odom[0] - d_goal[0]) + (d_odom[1] - d_goal[1])*(d_odom[1] - d_goal[1]));
    int ep_num = (int)floor(ep_len/MAX_SUBEP_LEN);

    double sin_alfa = (d_goal[1] - d_odom[1]) / ep_len;
    double cos_alfa = (d_goal[0] - d_odom[0]) / ep_len;

    double ep_dx = ep_len / ep_num * cos_alfa;
    double ep_dy = ep_len / ep_num * sin_alfa;


    for(int ep_no = 1; ep_no < ep_num; ep_no++){

        for(int y = 0; y < 8; y++){
            for(int x = 0; x < 8; x++){

                int sub_y = (int)(d_odom[1] + ep_no * ep_dy) - 4 + y;
                int sub_x = (int)(d_odom[0] + ep_no * ep_dx) - 4 + x;

                d_hmap[sub_y * 256 + sub_x] = 0;
            }
        }

        d_hmap[0] = 0;
        d_hmap[1] = 1;
        d_hmap[2] = 2;

    }

    *d_cost = d_odom[0];

    // *d_cost = *(d_odom+1) + *(d_goal+2) + *(d_hmap+3);
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
    gpuErrchk( cudaDeviceSynchronize() );

}

void GPUPathPlanner::gpuCopyInputToDevice(double *h_odom, double *h_goal, uint8_t *h_hmap){
    gpuErrchk( cudaMemcpy(d_odom, h_odom, 3*sizeof(double), cudaMemcpyHostToDevice) );
    gpuErrchk( cudaMemcpy(d_goal, h_goal, 3*sizeof(double), cudaMemcpyHostToDevice) );
    gpuErrchk( cudaMemcpy(d_hmap, h_hmap, hmap_rows*hmap_cols*sizeof(uint8_t), cudaMemcpyHostToDevice) );
    gpuErrchk( cudaDeviceSynchronize() );
}

void GPUPathPlanner::gpuExecuteKernel(){
    kernelPathPlanning<<<1,1>>>(d_odom, d_goal, d_hmap, d_cost);
    gpuErrchk( cudaPeekAtLastError() );
    gpuErrchk( cudaDeviceSynchronize() );
}


void GPUPathPlanner::gpuCopyOutputToHost(double *h_cost, uint8_t *h_hmap){
    gpuErrchk( cudaMemcpy(h_cost, d_cost, 1*sizeof(double), cudaMemcpyDeviceToHost) );
    gpuErrchk( cudaMemcpy(h_hmap, d_hmap, hmap_rows*hmap_cols*sizeof(uint8_t), cudaMemcpyDeviceToHost) );
    gpuErrchk( cudaDeviceSynchronize() );
}

void GPUPathPlanner::gpuFree(){
    gpuErrchk( cudaFree(d_odom) );
    gpuErrchk( cudaFree(d_goal) );
    gpuErrchk( cudaFree(d_hmap) );
    gpuErrchk( cudaFree(d_cost) );
}
