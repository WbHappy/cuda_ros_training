#include "../include/gpu_map_i16.cuh"

__global__ void fillValueKernel(int16_t* data, int16_t fill_value)
{
    int idx = blockDim.x * blockIdx.x + threadIdx.x;        // Globalny index x (kolumna na mapie wysokości)
    int idy = blockDim.y * blockIdx.y + threadIdx.y;        // Globalny index y (wiersz na mapie wysokości)
    int tid = idx + idy * gridDim.x * blockDim.x;           // Globalny numer indeksu (adres pamięci na mapie wysokości)

    data[tid] = fill_value;
}


GpuMapI16::GpuMapI16(int size_x, int size_y)
{
    this->resize(size_x, size_y);
}


GpuMapI16::GpuMapI16(int size_x, int size_y, const int16_t fill_value)
{
    this->resize(size_x, size_y);
    this->fill(fill_value);
}


void GpuMapI16::allocate(int size_x, int size_y)
{
    this->size_x = size_x;
    this->size_y = size_y;

    gpuErrchk( cudaMalloc((void**)&this->data, size_x * size_y * sizeof(int16_t)) );
}


void GpuMapI16::resize(int size_x, int size_y)
{
    release();
    allocate(size_x, size_y);
}


void GpuMapI16::fill(const int16_t fill_value)
{
    int block_x = 32;
    int block_y = 32;

    int grid_x = (size_x + block_x - 1) / block_x;
    int grid_y = (size_y + block_y - 1) / block_y;
    dim3 grid(grid_x, grid_y, 1);
    dim3 block(block_x, block_y, 1);

    fillValueKernel<<< grid, block >>> (this->data, fill_value);
    gpuErrchk( cudaPeekAtLastError() );
    gpuErrchk( cudaDeviceSynchronize() );

}


void GpuMapI16::release()
{
    gpuErrchk( cudaFree(this->data) );
}
