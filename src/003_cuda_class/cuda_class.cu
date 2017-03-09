#include "cuda_class.cuh"

__global__ void Kernel (uint32_t *d_a, uint32_t *d_b, uint32_t *d_c){
        uint32_t idx = blockDim.x * blockIdx.x + threadIdx.x;
        uint32_t idy = blockDim.y * blockIdx.y + threadIdx.y;
        uint32_t tid = idx + idy * blockDim.x * gridDim.x;

        d_c[tid] = d_a[tid] + d_b[tid];
};

CudaClass::CudaClass(uint32_t *h_a, uint32_t *h_b, uint32_t *h_c, uint32_t length){

    this->length = length;
    this->mem_size = length * sizeof(uint32_t);

    this->h_a = h_a;
    this->h_b = h_b;
    this->h_c = h_c;

    this->h_zeros = (uint32_t *) malloc(length*sizeof(uint32_t));

    for(int i = 0; i < length; i++){
        h_zeros[i] = 0;
    }

}

void CudaClass::allocateDeviceMemory(){
    cudaMalloc((void**) &d_a, mem_size);
    cudaMalloc((void**) &d_b, mem_size);
    cudaMalloc((void**) &d_c, mem_size);
}

void CudaClass::copyInputToDevice(){
    cudaMemcpy(d_a, h_a, mem_size, cudaMemcpyHostToDevice);
    cudaMemcpy(d_b, h_b, mem_size, cudaMemcpyHostToDevice);
}

void CudaClass::runKernelGPU(){
    // Kernel<<<length, 1>>>(d_a, d_b, d_c);
    Kernel<<<length, 1>>>(d_a, d_b, d_c);
}

void CudaClass::copyOutputToHost(){
    cudaMemcpy(h_c, d_c, mem_size, cudaMemcpyDeviceToHost);
}

void CudaClass::freeDeviceMemory(){
    cudaMemcpy(d_c, h_zeros, mem_size, cudaMemcpyHostToDevice);

    cudaFree(d_a);
    cudaFree(d_b);
    cudaFree(d_c);

}
