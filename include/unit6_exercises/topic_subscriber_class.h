#ifndef TOPIC_SUBSCRIBER_CLASS_H
#define TOPIC_SUBSCRIBER_CLASS_H

#include "ros/node_handle.h"
#include <ros/ros.h>
#include <string>

// NOTE: 
// 1. Class template.
// 2. Callback has different type parameter ROSCallbackMessageType to avoid shadowing class type parameter ROSMessageType.
// 3. Defintions in header, so empty source.

template <typename ROSMessageType>
class TopicSubscriber {
  ros::NodeHandle *nh_;
  std::string topic_name_;
  ros::Subscriber sub_;

  template <typename ROSCallbackMessageType>  
  void callback(const typename ROSCallbackMessageType::ConstPtr &msg) {
    ROS_INFO_STREAM("TopicSubscriber received image data (data[0] = " << static_cast<int>(msg->data[0]) << ")");
  }

public:
  TopicSubscriber(ros::NodeHandle *nh, std::string topic_name) {
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