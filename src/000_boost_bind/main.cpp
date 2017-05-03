#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <functional>

void function()
{
    ROS_INFO("Function executed");
}
std::function <void()> ftcl_function = function;


void callback(const std_msgs::Float64ConstPtr &msg)
{
    ROS_INFO("Callback executed, msg.data = %f", msg->data);
}

void callback2(const std_msgs::Float64ConstPtr &msg, int param)
{
    ROS_INFO("Callback executed, msg.data = %f", msg->data);
    ROS_INFO("Additional param %d", param);
}

void callback3(const std_msgs::Float64::ConstPtr &msg, std::function< void() > functocall)
{
    ROS_INFO("Callback executed, msg.data = %f", msg->data);
    functocall();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "boost_binsd_test");
    ros::NodeHandle nh;

    int param = 99;

    ros::Subscriber sub1 = nh.subscribe<std_msgs::Float64>("/some/topic1", 100, callback);
    ros::Subscriber sub2 = nh.subscribe<std_msgs::Float64>("/some/topic2", 100, boost::bind(callback2, _1, param));
    ros::Subscriber sub3 = nh.subscribe<std_msgs::Float64>("/some/topic3", 100, boost::bind(callback3, _1, function));
    ros::spin();

    return 0;
}
