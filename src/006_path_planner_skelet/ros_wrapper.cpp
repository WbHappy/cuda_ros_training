#include "ros_wrapper.hpp"

ROSWrapper::ROSWrapper(ros::NodeHandle *nh, image_transport::ImageTransport *it) : GPUAlgorithm(){
    this-> nh = nh;
    h_odom = (double*) malloc(3*sizeof(double));
    h_goal = (double*) malloc(3*sizeof(double));
    h_hmap_image = cv::Mat(256, 256, CV_8UC1);

    sub_odom = nh->subscribe("/cuda/odom", 100, &ROSWrapper::callbackOdom, this);
    sub_goal = nh->subscribe("/cuda/goal", 100, &ROSWrapper::callbackGoal, this);
    sub_hmap = it->subscribe("/cuda/hmap", 100, &ROSWrapper::callbackHmap, this);

    GPUAlgorithm::setupHost(this->h_odom, this->h_goal, this->h_hmap_image.data);
    GPUAlgorithm::setupCuda();

    printf ("=== %d === \n", h_hmap_image.data[12]);
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

    for(int i = 0 ; i < 1; i++){
        for(int j = 0; j < 256; j++){
            printf("%d\t", h_hmap_image.data[i*256+j]);
        }
        printf("\n");
    }

    GPUAlgorithm::copyInputToDevice(this->h_odom, this->h_goal, this->h_hmap_image.data);
    GPUAlgorithm::executeKernel();
    GPUAlgorithm::copyOutputToHost();

    for(int i = 0 ; i < 1; i++){
        for(int j = 0; j < 256; j++){
            printf("%d\t", GPUAlgorithm::h_hmap_buff[i*256+j]);
        }
        printf("\n");
    }
}
