#include "addition.cuh"

__global__ void addition_kernel(int *d_a, int *d_b, int *d_c){
    *d_c = *d_a + *d_b;
}

Addition::Addition(ros::NodeHandle *nh){
    this->nh = nh;

    this->sub_a = nh->subscribe("/cuda/input/a", 100, &Addition::callbackA, this);
    this->sub_b = nh->subscribe("/cuda/input/b", 100, &Addition::callbackB, this);
}

Addition::~Addition(){
    free(h_a);
    free(h_b);
    free(h_c);
}


void Addition::callbackA(const std_msgs::Int32 msg){
    *this->h_a = msg.data;

    copyInputToDevice();
    runKernelGPU();
    copyOutputToHost();

    ROS_INFO("C = %d", *h_c);
}

void Addition::callbackB(const std_msgs::Int32 msg){
    *this->h_b = msg.data;

    copyInputToDevice();
    runKernelGPU();
    copyOutputToHost();

    ROS_INFO("C = %d", *h_c);
}

void Addition::allocateDeviceMemory(){
    h_a = (int*) malloc(sizeof(int)); *h_a = 0;
    h_b = (int*) malloc(sizeof(int)); *h_b = 0;
    h_c = (int*) malloc(sizeof(int)); *h_c = 0;

    cudaMalloc((void**) &d_a, sizeof(int));
    cudaMalloc((void**) &d_b, sizeof(int));
    cudaMalloc((void**) &d_c, sizeof(int));
}

void Addition::copyInputToDevice(){
    cudaMemcpy(d_a, h_a, sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(d_b, h_b, sizeof(int), cudaMemcpyHostToDevice);
}

void Addition::runKernelGPU(){
    addition_kernel<<<1,1>>>(d_a, d_b, d_c);
}

void Addition::copyOutputToHost(){
    cudaMemcpy(h_c, d_c, sizeof(int), cudaMemcpyDeviceToHost);
}

void Addition::freeDeviceMemeory(){
    cudaFree(d_a);
    cudaFree(d_b);
    cudaFree(d_c);
}
