#include "addition.cuh"

#include <ros/ros.h>
#include <std_msgs/Int32.h>


int * A, * B, * C;
int * D_A, * D_B, * D_C;



void callbackA(const std_msgs::Int32 msg){
    *A = msg.data;
    copyInputToDevice(D_A, D_B, D_C, A, B, C);
    executeKernel(D_A, D_B, D_C, A, B, C);
    copyOutputToHost(D_A, D_B, D_C, A, B, C);
    ROS_INFO("C = %d | device", *C);
}

void callbackB(const std_msgs::Int32 msg){
    *B = msg.data;
    copyInputToDevice(D_A, D_B, D_C, A, B, C);
    executeKernel(D_A, D_B, D_C, A, B, C);
    copyOutputToHost(D_A, D_B, D_C, A, B, C);
    ROS_INFO("C = %d | device", *C);
}

int main(int argc, char** argv){

    // ROS_SETUP
    ros::init(argc,  argv, "topics_addition");
    ros::NodeHandle nh;

    A = (int*) malloc(sizeof(int)); *A = 8;
    B = (int*) malloc(sizeof(int)); *B = 4;
    C = (int*) malloc(sizeof(int)); *C = 0;


    // CUDA SETUP
    setupCuda(D_A, D_B, D_C, A, B, C);

    ros::Subscriber sub_a = nh.subscribe("/cuda/input/a", 100, &callbackA);
    ros::Subscriber sub_b = nh.subscribe("/cuda/input/b", 100, &callbackB);

    copyInputToDevice(D_A, D_B, D_C, A, B, C);
    executeKernel(D_A, D_B, D_C, A, B, C);
    copyOutputToHost(D_A, D_B, D_C, A, B, C);

    ROS_INFO("C = %d | device", *C);

    ros::spin();

    cleanupCuda(D_A, D_B, D_C);
    free(A);
    free(B);
    free(C);


    return 0;
}
