#ifndef IMAGEPUBLISHER_HPP_
#define IMAGEPUBLISHER_HPP_

#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Header.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sstream>



class ImagePublisher
{
protected:
    ros::NodeHandle* nh;
    image_transport::ImageTransport* it;
    image_transport::Publisher pub_img;

public:
    cv::Mat cv_img;
    sensor_msgs::ImagePtr img;
    std_msgs::Header header;

public:
    ImagePublisher(){}

    ImagePublisher(ros::NodeHandle* nh, image_transport::ImageTransport *it, int cam_num, std::string topic)
    {
        this->nh = nh;
        this->it = it;

        pub_img = it->advertise(topic, 1);

        header.seq = 0;
        header.frame_id = "";

        // cv_img = cv::Mat(256, 256, CV_8UC1);
        // for(int i = 0; i < 256*256; i++){
        //     *(cv_img.data + i) = 100;
        // }

        std::string path = ros::package::getPath("cuda_training");
        cv_img = cv::imread(path+"/src/009_path_planner_cost_map_local/pub_hmap/hmap1038.png" , cv::IMREAD_GRAYSCALE);

    }

    void publishImage()
    {
        header.seq++;
        header.stamp = ros::Time::now();

        // CameraHandler::Capture();

        // cv::waitKey(1);

        img = cv_bridge::CvImage(header, "mono8", cv_img).toImageMsg();

        pub_img.publish(img);
    }
};

#endif
