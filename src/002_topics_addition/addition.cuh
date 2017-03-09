#include <cuda.h>
#include <cuda_runtime.h>

void setupCuda(int* D_A, int* D_B, int* D_C, int* A, int* B, int* C);

void copyInputToDevice(int* D_A, int* D_B, int* D_C, int* A, int* B, int* C);

void executeKernel(int* D_A, int* D_B, int* D_C, int* A, int* B, int* C);

void copyOutputToHost(int* D_A, int* D_B, int* D_C, int* A, int* B, int* C);

void cleanupCuda(int* D_A, int* D_B, int* D_C);
