#include "ros_wrapper.hpp"
#include "gpu_path_planner.cuh"

int main(int argc, char** argv){

    ros::init(argc, argv, "path_planner_segment_cost");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    ROSWrapper gpu_path_planner(&nh, &it);

    ros::spin();

    return 0;
}
