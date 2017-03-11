#include "ros_wrapper.hpp"

ROSWrapper::ROSWrapper(ros::NodeHandle *nh, image_transport::ImageTransport *it) : GPUPathPlanner(){
    this-> nh = nh;
    h_odom = (double*) malloc(3*sizeof(double));
    h_goal = (double*) malloc(3*sizeof(double));
    h_hmap_image = cv::Mat(256, 256, CV_8UC1);
    h_cost = (double*) malloc(1*sizeof(double));

    sub_odom = nh->subscribe("/cuda/odom", 100, &ROSWrapper::callbackOdom, this);
    sub_goal = nh->subscribe("/cuda/goal", 100, &ROSWrapper::callbackGoal, this);
    sub_hmap = it->subscribe("/cuda/hmap", 100, &ROSWrapper::callbackHmap, this);
    pub_cost = nh->advertise<std_msgs::Float64>("/cuda/cost", 100);

    GPUPathPlanner::gpuSetup(256,256);
}


void ROSWrapper::callbackOdom(const nav_msgs::Odometry msg){
    this->h_odom[0] = msg.pose.pose.position.x;
    this->h_odom[1] = msg.pose.pose.position.y;
    this->h_odom[2] = msg.pose.pose.position.z;
}

void ROSWrapper::callbackGoal(const geometry_msgs::PoseStamped msg){
    this->h_goal[0] = msg.pose.position.x;
    this->h_goal[1] = msg.pose.position.y;
    this->h_goal[2] = msg.pose.position.z;
}

void ROSWrapper::callbackHmap(const sensor_msgs::ImageConstPtr& msg){
    this->h_hmap_image = cv_bridge::toCvShare(msg, "mono8")->image;

    GPUPathPlanner::gpuCopyInputToDevice(h_odom, h_goal, h_hmap_image.data);
    GPUPathPlanner::gpuExecuteKernel();
    GPUPathPlanner::gpuCopyOutputToHost(h_cost);

    ROS_INFO("cost: %f", *h_cost);
    msg_cost.data = *h_cost;
    pub_cost.publish(msg_cost);
}
