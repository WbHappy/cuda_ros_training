#include "dh/lidar_dh.hpp"

#include "gpu_maps/_robot_planner_maps.cuh"

#include "cuda_lidar_mapping.cuh"

#include "ros/_ros_buffor.hpp"

#include "ros/template_subscriber.hpp"
#include "ros/template_publisher.hpp"
#include "ros/utils.hpp"

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>

#include <functional>
#include <boost/bind.hpp>

bool NEW_LIDAR_SCAN = false;
int SCANS_FROM_LAST_MAPPING = 0;

void function_newScan()
{
    NEW_LIDAR_SCAN = true;
    SCANS_FROM_LAST_MAPPING++;
}


int main(int argc, char** argv)
{
    _RobotPlannerMaps _RPM;
    _ROSBuffor _ROSBUFF;

    CudaLidarMapping CLM(&_RPM, &_ROSBUFF);
    LidarDH LDH;

    ros::init(argc, argv, "cuda_lidar_mapping");
    ros::NodeHandle nh;
    ros::Rate wait_loop_rate(15);
    ros::Rate main_loop_rate(15);
    std::string node_name = ros::this_node::getName();

    std::string goal_topic, odom_topic, lidar_enc_topic, lidar_scan_topic;
    nh.param(node_name + "/goal_topic", goal_topic, std::string("navigation/goal"));
    nh.param(node_name + "/odom_topic", odom_topic, std::string("navigation/odom_ekf"));
    nh.param(node_name + "/lidar_enc_topic", lidar_enc_topic, std::string("encoder/lidar_tower_abs/pose"));
    nh.param(node_name + "/lidar_scan_topic", lidar_scan_topic, std::string("lidar"));

    TemplateSubscriber <geometry_msgs::PoseStamped> sub_goal(&nh , goal_topic, &_ROSBUFF.goal);
    TemplateSubscriber <nav_msgs::Odometry> sub_odom(&nh , odom_topic, &_ROSBUFF.odom);
    TemplateSubscriber <std_msgs::Float64> sub_lidar_pose(&nh , lidar_enc_topic, &_ROSBUFF.lidar_pose);
    TemplateSubscriber <sensor_msgs::LaserScan> sub_lidar_scan(&nh , lidar_scan_topic, &_ROSBUFF.laser_scan, function_newScan);

    utils::waitFor(&sub_goal.msg_recived, &wait_loop_rate, "waiting for goal message");

    _RPM.allocateMaps(_ROSBUFF.goal.pose.position.x, _ROSBUFF.goal.pose.position.y);
    utils::waitFor(&sub_odom.msg_recived, &wait_loop_rate, "waiting for odom message");
    utils::waitFor(&sub_lidar_pose.msg_recived, &wait_loop_rate, "waiting for lidar encoder pose message");
    utils::waitFor(&sub_lidar_scan.msg_recived, &wait_loop_rate, "waiting for lidar scan message");

    _RPM.allocateLaserScan(_ROSBUFF.laser_scan.ranges.size());

    while(ros::ok())
    {
        ros::spinOnce();

        _RPM.dk_matrix= LDH.dkWorldToLidar(
                                _ROSBUFF.odom.pose.pose.position.x,
                                _ROSBUFF.odom.pose.pose.position.y,
                                _ROSBUFF.odom.pose.pose.position.z,
                                _ROSBUFF.odom.pose.pose.orientation.x,
                                _ROSBUFF.odom.pose.pose.orientation.y,
                                _ROSBUFF.odom.pose.pose.orientation.z,
                                _ROSBUFF.odom.pose.pose.orientation.w,
                                _ROSBUFF.lidar_pose.data);

        CLM.copyInputToDevice();
        CLM.executeKernel();
        CLM.copyOutputToHost();
        CLM.display();

        _ROSBUFF.debugInfo();
        _RPM.cudaDebugInfo();

        main_loop_rate.sleep();
    }


    return 0;
}
