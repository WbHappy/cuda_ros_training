#include "../include/_robot_planner_maps.cuh"
#include "../include/ros/_ros_buffor.hpp"

#include "../include/gpu_001_lidar_mapping.cuh"
#include "../include/gpu_002_cost_mapping.cuh"
#include "../include/gpu_003_path_planning.cuh"

#include "../include/ht_matrix.hpp"

#include "../include/ros/template_subscriber.hpp"
#include "../include/ros/template_publisher.hpp"
#include "../include/ros/ros_utils.hpp"

_RobotPlannerMaps _RPM;
_ROSBuffor _ROSBUFF;

GpuLidarMapping GLM(&_RPM, &_ROSBUFF);
GpuCostMapping GCM(&_RPM, &_ROSBUFF);


void updateRobotPoseOnMap()
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


    nh.param(node_name + "/height_scale", _RPM.height_scale, (int) 100);
    nh.param(node_name + "/map_scale", _RPM.map_scale, (int) 10 );
    nh.param(node_name + "/map_pow2_divider", _RPM.map_pow2_divider, (int) 32);
    nh.param(node_name + "/map_meters_offset", _RPM.map_meters_offset, (int) 15);


    nh.param(node_name + "/dk_a1", GLM.dk_a1, (float) -0.2);
    nh.param(node_name + "/dk_d2", GLM.dk_d2, (float) 0.5);
    nh.param(node_name + "/dk_al3", GLM.dk_al3, (float) 0.45);
    nh.param(node_name + "/init_circle_height", GLM.init_circle_height, (int) -40);
    nh.param(node_name + "/init_circle_radius", GLM.init_circle_radius, (float) 25.0);


    nh.param(node_name + "/cmap_refresh_radius_meters", GCM.cmap_refresh_radius_meters, (float) 15);
        GCM.cmap_refresh_radius_pix = (int)(GCM.cmap_refresh_radius_meters * _RPM.map_scale);
    nh.param(node_name + "/cost_mask_radius", GCM.cost_mask_radius, (int) 16);
    nh.param(node_name + "/unknown_field_cost", GCM.unknown_field_cost, (int) 1);
    nh.param(node_name + "/costmap_borders_value", GCM.costmap_borders_value, (int) 1000);

    // ROS Communication
    TemplateSubscriber <geometry_msgs::PoseStamped> sub_goal(&nh , goal_topic, &_ROSBUFF.goal);
    TemplateSubscriber <nav_msgs::Odometry> sub_odom(&nh , odom_topic, &_ROSBUFF.odom, updateRobotPoseOnMap);
    TemplateSubscriber <std_msgs::Float64> sub_lidar_pose(&nh , lidar_enc_topic, &_ROSBUFF.lidar_pose);
    TemplateSubscriber <sensor_msgs::LaserScan> sub_lidar_scan(&nh , lidar_scan_topic, &_ROSBUFF.laser_scan);


    // Waiting for goal message
    utils::waitFor(&sub_goal.msg_recived, &wait_loop_rate, "waiting for goal message");
    // Allocating maps in CPU and GPU memory
    _RPM.allocateMaps(_ROSBUFF.goal.pose.position.x, _ROSBUFF.goal.pose.position.y);
    // Drawing circle in 0 pose of robot on heightmap - terrain which cannot be mapped before moving
    GLM.drawInitialHeightmapCircle();
    // Drawing borders of costmap - this terrain should not be allowed for rover
    GCM.drawInitialCostmapBorders();

    // Waiting lidar scan messages
    utils::waitFor(&sub_lidar_scan.msg_recived, &wait_loop_rate, "waiting for lidar scan message");
    // Allocating lidar scan array in CPU and GPU memory
    GLM.allocateMemory(_ROSBUFF.laser_scan.ranges.size(), _ROSBUFF.laser_scan.angle_min, _ROSBUFF.laser_scan.angle_max);

    // Waiting for odom, lidar_pose
    utils::waitFor(&sub_odom.msg_recived, &wait_loop_rate, "waiting for odom message");
    utils::waitFor(&sub_lidar_pose.msg_recived, &wait_loop_rate, "waiting for lidar encoder pose message");


    while(ros::ok())
    {
        ros::spinOnce();

        _RPM.updateRobotPoseOnMap(_ROSBUFF.odom.pose.pose.position.x, _ROSBUFF.odom.pose.pose.position.y);

        GLM.copyInputToDevice();
        GLM.executeKernel();
        GLM.copyOutputToHost();
        GLM.display();


        GCM.copyInputToDevice();
        GCM.executeKernel();
        GCM.copyOutputToHost();
        GCM.display();




        main_loop_rate.sleep();
    }

    return 0;

}
