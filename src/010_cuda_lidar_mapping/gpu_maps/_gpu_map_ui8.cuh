#ifndef GPU_MAP_UI8_CUH_
#define GPU_MAP_UI8_CUH_

#include <cuda.h>
#include "../gpu_errchk/gpu_errchk.cuh"

class _GpuMap_UI8
{
    int size_x;
    int size_y;

public:
    uint8_t* data;



public:

    _GpuMap_UI8(){};
    _GpuMap_UI8(int size_x, int size_y);
    _GpuMap_UI8(int size_x, int size_y, const uint8_t fill_value);
    ~_GpuMap_UI8();

    void allocate(int size_x, int size_y);
    void resize(int size_x, int size_y);
    void fill(const uint8_t fill_value);
    void free();

};

#endif
