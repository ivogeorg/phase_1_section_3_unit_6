#include "ros/init.h"
#include "unit6_exercises/topic_subscriber_function.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>

int main (int argc, char **argv) {
    ros::init(argc, argv, "topic_subscriber_function_node");
    ros::NodeHandle nh("topic_subscriber_function_nh");
    std::string topic_name("/camera/depth_registered/points");

    TopicSubscriber topic_sub;
   topic_sub.init<sensor_msgs::PointCloud2>(&nh, topic_name);

    ros::spin();
    
    return 0;
}