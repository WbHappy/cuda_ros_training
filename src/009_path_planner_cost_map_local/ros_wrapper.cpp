#include "ros_wrapper.hpp"

ROSWrapper::ROSWrapper(ros::NodeHandle *nh, image_transport::ImageTransport *it) : GPUPathPlanner(MASK_SIZE, LASER_RANGE)
{
    this-> nh = nh;
    h_odom = (double*) malloc(3*sizeof(double));
    h_goal = (double*) malloc(3*sizeof(double));
    // h_hmap_image = cv::Mat(HMAP_DIM_ROWS, HMAP_DIM_COLS, CV_8UC1);
    // h_cmap_image = cv::Mat(CMAP_DIM_ROWS, CMAP_DIM_COLS, CV_8UC1);
    h_debug = (double*) malloc(32*sizeof(double));

    this->h_odom[0] = 256.0;
    this->h_odom[1] = 256.0;
    this->h_odom[2] = 30.0;
    this->h_goal[0] = 0.0;
    this->h_goal[1] = 0.0;
    this->h_goal[2] = 0.0;

    hmap_recived = false;

    // GPUPathPlanner::gpuSetup(HMAP_DIM_ROWS,HMAP_DIM_COLS);
    // ROS_INFO("MemoryAlocated");

    sub_odom = nh->subscribe("/cuda/odom", 100, &ROSWrapper::callbackOdom, this);
    sub_goal = nh->subscribe("/cuda/goal", 100, &ROSWrapper::callbackGoal, this);
    sub_hmap = it->subscribe("/cuda/hmap", 100, &ROSWrapper::callbackHmap, this);
    pub_debug = nh->advertise<std_msgs::Float64>("/cuda/debug", 100);


    cv::namedWindow("win1");
    cv::namedWindow("win2");

    stopwatch = Stopwatch();
}


void ROSWrapper::callbackOdom(const nav_msgs::Odometry msg)
{
    this->h_odom[0] = msg.pose.pose.position.x;
    this->h_odom[1] = msg.pose.pose.position.y;
    this->h_odom[2] = msg.pose.pose.position.z;
    ROS_WARN("Odom: %f %f %f", h_odom[0], h_odom[1], h_odom[2]);

}

void ROSWrapper::callbackGoal(const geometry_msgs::PoseStamped msg)
{
    this->h_goal[0] = msg.pose.position.x;
    this->h_goal[1] = msg.pose.position.y;
    this->h_goal[2] = msg.pose.position.z;
    ROS_WARN("Goal: %f %f %f", h_goal[0], h_goal[1], h_goal[2]);

}

void ROSWrapper::callbackHmap(const sensor_msgs::ImageConstPtr& msg)
{

    if(hmap_recived == false)
    {
        hmap_recived = true;

        stopwatch.Start();

        h_hmap_image = cv::Mat(msg->height, msg->width, CV_8UC1);
        h_cmap_image = cv::Mat(msg->height - 2*MASK_OFFSET, msg->width - 2*MASK_OFFSET, CV_8UC1);

        GPUPathPlanner::gpuSetup(msg->height, msg->width);
        GPUPathPlanner::gpuClearMemory();

        stopwatch.Check("Memory Allocation");

    }

    this->h_hmap_image = cv_bridge::toCvShare(msg, "mono8")->image;

    cv::imshow("win1", this->h_hmap_image);
    cv::waitKey(10);

    stopwatch.Start();

    GPUPathPlanner::gpuCopyHMapToDevice(h_hmap_image.data);
    GPUPathPlanner::gpuCopyOdomToDevice(h_odom);
    GPUPathPlanner::gpuCopyGoalToDevice(h_goal);

    GPUPathPlanner::gpuExecuteKernel();

    GPUPathPlanner::gpuCopyCMapFromDevice(h_cmap_image.data);
    GPUPathPlanner::gpuCopyDebugFromDevice(h_debug);

    stopwatch.Check("One 1024x1024 map executed with variance 15x15 mask");


    cv::imshow("win2", this->h_cmap_image);
    cv::waitKey(10);
    ROS_INFO("debug: %f", *h_debug);
    msg_debug.data = *h_debug;
    pub_debug.publish(msg_debug);
}
