#pragma once

#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <tbb/tbb.h>

class PoseSubscriberBase{
public:
    tbb::concurrent_bounded_queue<geometry_msgs::PoseWithCovarianceStamped> pose_buffer;
};

template<typename T>
class PoseSubscriber : public PoseSubscriberBase{

public:

    // no const before nodehandle!
    PoseSubscriber(ros::NodeHandle& nh, const std::string& topic_name, size_t buffer_size = 3) : 
        sub_(nh.subscribe(topic_name, buffer_size, &PoseSubscriber<T>::callback, this))
    {
        std::cout << "Subscribed to " << topic_name << std::endl;
    }

private:
    ros::Subscriber sub_;

    // qualified dependent name needs typename keyword
    void callback(const typename T::ConstPtr);
};

template<> void PoseSubscriber<geometry_msgs::PoseWithCovarianceStamped>::callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg)
{
    // std::cout << "Received message " << msg->header.stamp << std::endl;
    pose_buffer.push(*msg);
}

template<> void PoseSubscriber<geometry_msgs::PoseStamped>::callback(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    // std::cout << "Received message " << msg->header.stamp << std::endl;
    geometry_msgs::PoseWithCovarianceStamped out_msg;
    out_msg.header = msg->header;
    out_msg.pose.pose = msg->pose;
    pose_buffer.push(out_msg);
}

template<> void PoseSubscriber<nav_msgs::Odometry>::callback(const nav_msgs::Odometry::ConstPtr msg)
{
    // std::cout << "Received message " << msg->header.stamp << std::endl;
    geometry_msgs::PoseWithCovarianceStamped out_msg;
    out_msg.header = msg->header;
    out_msg.pose = msg->pose;
    pose_buffer.push(out_msg);
}