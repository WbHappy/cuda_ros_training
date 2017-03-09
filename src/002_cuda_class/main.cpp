#include "cuda_class.cuh"

#include <ros/ros.h>

#define N 16

int main(int argc, char** argv){

    ros::init(argc,  argv, "topics_addition_class");
    ros::NodeHandle nh;

    uint32_t* h_a = (uint32_t*) malloc(N*sizeof(uint32_t));
    uint32_t* h_b = (uint32_t*) malloc(N*sizeof(uint32_t));
    uint32_t* h_c = (uint32_t*) malloc(N*sizeof(uint32_t));

    for(int i = 0; i < N; i++){
        h_a[i] = i;
        h_b[i] = 4*i;
    }

    CudaClass cuda_object(h_a, h_b, h_c, N);

    cuda_object.AllocateDeviceMemory();
    cuda_object.CopyInputToDevice();
    cuda_object.RunKernelGPU();
    cuda_object.CopyOutputToHost();
    cuda_object.FreeDeviceMemory();

    for(int i = 0; i < N; i++){
        printf("%d\n", h_c[i]);
    }

    free(h_a);
    free(h_b);
    free(h_c);

    return 0;

 }
