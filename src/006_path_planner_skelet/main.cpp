#include "ros_wrapper.hpp"
#include "gpu_algorithm.cuh"

int main(int argc, char** argv){

    ros::init(argc, argv, "path_planner_skelet");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    ROSWrapper gpu_ros(&nh, &it);

    ros::spin();

    return 0;
}
