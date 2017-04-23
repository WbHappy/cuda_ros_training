#include "dh/lidar_dh.hpp"

#include "gpu_maps/_robot_planner_maps.hpp"

#include "cuda_lidar_mapping.cuh"

#include "ros/template_subscriber.hpp"
#include "ros/template_publisher.hpp"
#include "ros/utils.hpp"

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>

void lidarScanCallback(CudaLidarMapping *clm)
{
    clm->printAqq();
}

int main(int argc, char** argv)
{

    _RobotPlannerMaps _rpm;
    CudaLidarMapping cuda_lidar_mapping(&_rpm);
    LidarDH lidar_transform;


    ros::init(argc, argv, "cuda_lidar_mapping");
    ros::NodeHandle nh;
    ros::Rate wait_loop_rate(10);
    ros::Rate main_loop_rate(10);
    std::string node_name = ros::this_node::getName();

    std::string goal_topic, odom_topic, lidar_enc_topic, lidar_scan_topic;
    nh.param(node_name + "/goal_topic", goal_topic, std::string("navigation/goal"));
    nh.param(node_name + "/odom_topic", odom_topic, std::string("navigation/odom_ekf"));
    nh.param(node_name + "/lidar_enc_topic", lidar_enc_topic, std::string("encoder/lidar_tower_abs/pose"));
    nh.param(node_name + "/lidar_scan_topic", lidar_scan_topic, std::string("lidar"));

    TemplateSubscriber <geometry_msgs::PoseStamped>sub_goal(&nh , goal_topic);
    TemplateSubscriber <nav_msgs::Odometry>sub_odom(&nh , odom_topic);
    TemplateSubscriber <std_msgs::Float64>sub_lidar_pose(&nh , lidar_enc_topic);
    TemplateSubscriber <sensor_msgs::LaserScan>sub_lidar_scan(&nh , lidar_scan_topic);


    utils::waitFor(&sub_goal.msg_recived, &wait_loop_rate, "waiting for goal message");

    _rpm.allocateMaps(sub_goal.msg.pose.position.x, sub_goal.msg.pose.position.y);

    utils::waitFor(&sub_odom.msg_recived, &wait_loop_rate, "waiting for odom message");
    utils::waitFor(&sub_lidar_pose.msg_recived, &wait_loop_rate, "waiting for lidar encoder pose message");
    utils::waitFor(&sub_lidar_scan.msg_recived, &wait_loop_rate, "waiting for lidar scan message");

    while(ros::ok())
    {
        ros::spinOnce();

        _rpm.dk_matrix= lidar_transform.dkWorldToLidar(
                            sub_odom.msg.pose.pose.position.x,
                            sub_odom.msg.pose.pose.position.y,
                            sub_odom.msg.pose.pose.position.z,
                            sub_odom.msg.pose.pose.orientation.x,
                            sub_odom.msg.pose.pose.orientation.y,
                            sub_odom.msg.pose.pose.orientation.z,
                            sub_odom.msg.pose.pose.orientation.w,
                            sub_lidar_pose.msg.data);
        //



        main_loop_rate.sleep();
    }


    return 0;
}
