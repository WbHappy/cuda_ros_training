#ifndef ROS_WRAPPER_HPP_
#define ROS_WRAPPER_HPP_

#include "gpu_path_planner.cuh"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <stdio.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float64.h>

class ROSWrapper : public GPUPathPlanner{

    ros::NodeHandle *nh;

    ros::Subscriber sub_odom;
    ros::Subscriber sub_goal;

    image_transport::Subscriber sub_hmap;

    ros::Publisher pub_cost;
    std_msgs::Float64 msg_cost;

public:
    double *h_odom;
    double *h_goal;
    cv::Mat h_hmap_image;
    double *h_cost;

public:
    ROSWrapper(ros::NodeHandle *nh, image_transport::ImageTransport *it);

    void callbackOdom(const nav_msgs::Odometry msg);
    void callbackGoal(const geometry_msgs::PoseStamped msg);
    void callbackHmap(const sensor_msgs::ImageConstPtr& msg);

};


#endif
