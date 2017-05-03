#include "../include/_robot_planner_maps.cuh"
#include "../include/ros/_ros_buffor.hpp"

#include "../include/gpu_001_lidar_mapping.cuh"
#include "../include/gpu_002_cost_mapping.cuh"
#include "../include/gpu_003_path_planning.cuh"

#include "../include/ht_matrix.hpp"

#include "../include/ros/template_subscriber.hpp"
#include "../include/ros/template_publisher.hpp"
#include "../include/ros/utils.hpp"

_RobotPlannerMaps _RPM;
_ROSBuffor _ROSBUFF;

GpuLidarMapping GLM(&_RPM, &_ROSBUFF);


void fncNewLidaraScan()
{

}

int main(int argc, char** argv)
{

    // ROS initialization
    ros::init(argc, argv, "cuda_lidar_mapping");
    ros::NodeHandle nh;
    ros::Rate wait_loop_rate(15);
    ros::Rate main_loop_rate(15);
    std::string node_name = ros::this_node::getName();

    // Configureable parameters
    std::string goal_topic, odom_topic, lidar_enc_topic, lidar_scan_topic;
    nh.param(node_name + "/goal_topic", goal_topic, std::string("/kalman/simulation/navigation/goal"));
    nh.param(node_name + "/odom_topic", odom_topic, std::string("/kalman/simulation/navigation/perfect_odometry"));
    nh.param(node_name + "/lidar_enc_topic", lidar_enc_topic, std::string("/kalman/simulation/encoder/lidar_tower_abs/pose"));
    nh.param(node_name + "/lidar_scan_topic", lidar_scan_topic, std::string("/kalman/simulation/lidar"));

    // ROS Communication
    TemplateSubscriber <geometry_msgs::PoseStamped> sub_goal(&nh , goal_topic, &_ROSBUFF.goal);
    TemplateSubscriber <nav_msgs::Odometry> sub_odom(&nh , odom_topic, &_ROSBUFF.odom);
    TemplateSubscriber <std_msgs::Float64> sub_lidar_pose(&nh , lidar_enc_topic, &_ROSBUFF.lidar_pose);
    TemplateSubscriber <sensor_msgs::LaserScan> sub_lidar_scan(&nh , lidar_scan_topic, &_ROSBUFF.laser_scan, fncNewLidaraScan);


    // Allocating const size memory blocks in CPU and GPU (like DH matrix)
    _RPM.allocateConstSizeMemory();

    // Waiting for goal message
    utils::waitFor(&sub_goal.msg_recived, &wait_loop_rate, "waiting for goal message");
    // Allocating maps in CPU and GPU memory
    _RPM.allocateMaps(_ROSBUFF.goal.pose.position.x, _ROSBUFF.goal.pose.position.y);


    // Waiting lidar scan messages
    utils::waitFor(&sub_lidar_scan.msg_recived, &wait_loop_rate, "waiting for lidar scan message");
    // Allocating lidar scan array in CPU and GPU memory
    _RPM.allocateLaserScan(_ROSBUFF.laser_scan.ranges.size());

    // Waiting for odom, lidar_pose
    utils::waitFor(&sub_odom.msg_recived, &wait_loop_rate, "waiting for odom message");
    utils::waitFor(&sub_lidar_pose.msg_recived, &wait_loop_rate, "waiting for lidar encoder pose message");


    while(ros::ok())
    {
        ros::spinOnce();

        GLM.copyInputToDevice();
        GLM.executeKernel();
        GLM.copyOutputToHost();

        GLM.display();

        main_loop_rate.sleep();
    }

}
