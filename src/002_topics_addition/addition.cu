#include "addition.cuh"
#include <stdio.h>

__global__ void addition(int* a, int* b, int* c){
    *c = *a + *b;
}

void setupCuda(int* D_A, int* D_B, int* D_C, int* A, int* B, int* C){
    cudaMalloc((void**)&D_A, sizeof(int));
    cudaMalloc((void**)&D_B, sizeof(int));
    cudaMalloc((void**)&D_C, sizeof(int));

    cudaMemcpy(D_A, A, sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(D_B, B, sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(D_C, C, sizeof(int), cudaMemcpyHostToDevice);
}

void copyInputToDevice(int* D_A, int* D_B, int* D_C, int* A, int* B, int* C){
    cudaMemcpy(D_A, A, sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(D_B, B, sizeof(int), cudaMemcpyHostToDevice);
}

void executeKernel(int* D_A, int* D_B, int* D_C, int* A, int* B, int* C){
    addition<<<1,1>>>(D_A, D_B, D_C);
}

void copyOutputToHost(int* D_A, int* D_B, int* D_C, int* A, int* B, int* C){
    cudaMemcpy(C, D_C, sizeof(int), cudaMemcpyDeviceToHost);
}

void cleanupCuda(int* D_A, int* D_B, int* D_C){
    cudaFree(D_A);
    cudaFree(D_B);
    cudaFree(D_C);
}
