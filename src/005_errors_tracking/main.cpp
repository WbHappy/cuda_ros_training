#include "addition.cuh"

int main(int argc, char** argv){

    ros::init(argc, argv, "topics_addition_class");
    ros::NodeHandle nh;

    Addition addition(&nh);

    addition.allocateDeviceMemory();

    ros::spin();

    addition.freeDeviceMemeory();

    return 0;
}
