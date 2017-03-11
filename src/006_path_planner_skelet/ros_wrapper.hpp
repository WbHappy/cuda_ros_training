#ifndef ROS_WRAPPER_HPP_
#define ROS_WRAPPER_HPP_

#include "gpu_algorithm.cuh"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <stdio.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>

class ROSWrapper : public GPUAlgorithm{

    ros::NodeHandle *nh;

    ros::Subscriber sub_odom;
    ros::Subscriber sub_goal;

    image_transport::Subscriber sub_hmap;
public:
    double *h_odom;
    double *h_goal;

    cv::Mat h_hmap_image;

public:
    ROSWrapper(ros::NodeHandle *nh, image_transport::ImageTransport *it);

    void callbackOdom(const nav_msgs::Odometry msg);
    void callbackGoal(const geometry_msgs::PoseStamped msg);
    void callbackHmap(const sensor_msgs::ImageConstPtr& msg);

};


#endif
