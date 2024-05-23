#ifndef TOPIC_SUBSCRIBER_FUNCTION_H
#define TOPIC_SUBSCRIBER_FUNCTION_H

#include "ros/node_handle.h"
#include <ros/ros.h>
#include <string>

// NOTE: 
// 1. Only the functions are templates.
// 2. Function definitions in header, so empty source.
// 3. Basic constructor and destructor.

class TopicSubscriber {
  ros::NodeHandle *nh_;
  std::string topic_name_;
  ros::Subscriber sub_;

  template <typename ROSMessageType>
  void callback(const typename ROSMessageType::ConstPtr &msg) {
    ROS_INFO_STREAM("TopicSubscriber received image data (data[0] = " << static_cast<int>(msg->data[0]) << ")");
  }

public:
  TopicSubscriber() { ROS_INFO_STREAM("TopicSubscriber constructor executed"); }
  ~TopicSubscriber() { ROS_INFO_STREAM("TopicSubscriber destructor executed"); }

  template <typename ROSMessageType>
  void init(ros::NodeHandle *nh, std::string topic_name) {
    nh_ = nh;
    topic_name_ = topic_name;

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                       ros::console::levels::Info)) {
      ros::console::notifyLoggerLevelsChanged();
    }

    sub_ = nh_->subscribe(topic_name_, 1, &TopicSubscriber::callback<ROSMessageType>, this);
  }
};

#endif