#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <ros/ros.h>

namespace utils
{



    void waitFor(bool *value, ros::Rate* rate)
    {
        while(ros::ok())
        {
            ros::spinOnce();
            if(*value)
            {
                break;
            }
            rate->sleep();
        }
    }

    void waitFor(bool *value, ros::Rate* rate, std::string msg)
    {
        while(ros::ok())
        {
            ros::spinOnce();
            if(*value)
            {
                break;
            }
            ROS_WARN_STREAM(msg);
            rate->sleep();
        }
    }



};

#endif
