#include <ros/ros.h>
#include <cuda.h>

__global__ void mykernel(int* device_a){
    *device_a = 22;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "hello_world");
    ros::NodeHandle nh;

    int* host_a = (int*) malloc(sizeof(int));
    int* device_a; cudaMalloc((void**) &device_a, sizeof(int));
    cudaMemcpy(device_a, host_a, sizeof(int), cudaMemcpyHostToDevice);
    mykernel<<<1,1>>>(device_a);
    cudaMemcpy(host_a, device_a, sizeof(int), cudaMemcpyDeviceToHost);

    ROS_INFO("Hello World! %d", *host_a);

    cudaFree(device_a);
    free(host_a);
    return 0;
}
