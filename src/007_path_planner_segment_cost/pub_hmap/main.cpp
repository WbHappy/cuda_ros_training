#include <ros/ros.h>
#include "image_publisher.hpp"

int main(int argc, char** argv){

    ros::init(argc, argv, "path_planner_skelet_pub_hmap");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    ImagePublisher image_publisher(&nh, &it, 0, "/cuda/hmap");


    ros::Rate loop_rate(1);

    while(ros::ok()){
        ros::spinOnce();
        image_publisher.publishImage();

        loop_rate.sleep();

    }


    return 0;

}
