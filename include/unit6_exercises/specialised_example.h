#ifndef MAGIC_SUBSCRIBER_H
#define MAGIC_SUBSCRIBER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;
using sensor_msgs::Image;
using sensor_msgs::PointCloud2;

template <typename ROSMessageType> class MagicSubscriber {
public:
  MagicSubscriber(ros::NodeHandle &ros_node, const string subscriber_topic);
  ~MagicSubscriber();
  void CallbackToTopic(const typename ROSMessageType::ConstPtr &msg);

private:
  ros::NodeHandle *m_ros_node_object;
  string m_subscriber_topic;
  ros::Subscriber image_sub_;
};

// function Definition

template <typename ROSMessageType>
MagicSubscriber<ROSMessageType>::MagicSubscriber(
    ros::NodeHandle &ros_node, const string subscriber_topic) {
  // We used an initialiser list
  this->m_subscriber_topic = subscriber_topic;
  this->m_ros_node_object = &ros_node;

  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Info)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  image_sub_ = this->m_ros_node_object->subscribe(
      this->m_subscriber_topic, 1,
      &MagicSubscriber<ROSMessageType>::CallbackToTopic, this);
};

template <typename ROSMessageType>
MagicSubscriber<ROSMessageType>::~MagicSubscriber() {
  cout << "MagicSubscriber Destructor is called" << endl;
};

template <typename ROSMessageType>
void MagicSubscriber<ROSMessageType>::CallbackToTopic(
    const typename ROSMessageType::ConstPtr &msg) {
  // the uint8 is an alias of unsigned char, therefore needs casting to int
  ROS_INFO_STREAM("GENERIC Template Callback message =" << msg);
};

template <>
inline void
MagicSubscriber<Image>::CallbackToTopic(const Image::ConstPtr &msg) {
  // the uint8 is an alias of unsigned char, therefore needs casting to int
  ROS_INFO_STREAM("Specialised IMAGE Template Callback Data[0]="
                  << static_cast<int>(msg->data[0]));
};

template <>
inline void MagicSubscriber<PointCloud2>::CallbackToTopic(
    const PointCloud2::ConstPtr &msg) {
  // the uint8 is an alias of unsigned char, therefore needs casting to int
  ROS_INFO_STREAM("Specialised POINTCLOUD Template Callback Data[0]="
                  << static_cast<int>(msg->data[0]));
};

#endif