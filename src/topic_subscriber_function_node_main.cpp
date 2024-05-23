#include "ros/init.h"
#include "unit6_exercises/topic_subscriber_function.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <string>

int main (int argc, char **argv) {
    ros::init(argc, argv, "topic_subscriber_function_node");
    ros::NodeHandle nh("topic_subscriber_function_nh");
    std::string topic_name("/camera/rgb/image_raw");

    TopicSubscriber topic_sub;
    // topic_sub.init(&nh, topic_name);
    topic_sub.init<sensor_msgs::Image>(&nh, topic_name);

    ros::spin();
    
    return 0;
}