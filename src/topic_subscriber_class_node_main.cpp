#include "ros/init.h"
#include "unit6_exercises/topic_subscriber_class.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <string>

int main (int argc, char **argv) {
    ros::init(argc, argv, "topic_subscriber_class_node");
    ros::NodeHandle nh("topic_subscriber_class_nh");
    std::string topic_name("/camera/rgb/image_raw");

    TopicSubscriber<sensor_msgs::Image> topic_sub(&nh, topic_name);

    ros::spin();
    
    return 0;
}