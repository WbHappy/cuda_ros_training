#ifndef TEMPLATE_SUBSCRIBER_HPP_
#define TEMPLATE_SUBSCRIBER_HPP_

#include <ros/ros.h>
#include <std_msgs/Int32.h>

template <typename ros_Message>
class TemplateSubscriber
{
protected:
    ros::NodeHandle *nh;
    ros::Subscriber sub;
public:
    ros_Message msg;
    bool msg_recived;


public:
    TemplateSubscriber(ros::NodeHandle *nh, std::string topic){
        this->msg_recived = false;
        this->nh = nh;
        this->sub = nh->subscribe(topic, 100, &TemplateSubscriber::msgInterrupt, this);
    }

    TemplateSubscriber(ros::NodeHandle *nh, std::string topic, void *function){
        this->msg_recived = false;
        this->nh = nh;
        this->sub = nh->subscribe(topic, 100, &TemplateSubscriber::msgInterrupt, this);
    }

    ~TemplateSubscriber(){}

    void msgInterrupt(const ros_Message msg){
        this->msg = msg;
        this->msg_recived = true;
    }

    bool firstMsgRecived(){
        return msg_recived;
    }
};

#endif
