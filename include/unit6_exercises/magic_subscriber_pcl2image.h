#ifndef MAGIC_SUBSCRIBER_H
#define MAGIC_SUBSCRIBER_H

// ROS core
#include "unit6_exercises/magic_subscriber_image_edgedetector.h"
#include <ros/ros.h>
// Image message
#include <sensor_msgs/Image.h>
// PointCloud message
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
// pcl::toROSMsg
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud_conversion.h>
// OpenCV bridge
#include <cv_bridge/cv_bridge.h>
// STL
#include <string>

template <typename ROSMessageType> class MagicSubscriber {
  ros::NodeHandle *nh_;
  std::string sub_topic_;
  std::string pub_topic_;
  ros::Subscriber image_sub_;
  ros::Publisher image_pub_;

  void CallbackToTopic(const typename ROSMessageType::ConstPtr &msg);

public:
  MagicSubscriber(ros::NodeHandle &ros_node, const std::string subscriber_topic,
                  const std::string publisher_topic);
  ~MagicSubscriber();
};

template <typename ROSMessageType>
MagicSubscriber<ROSMessageType>::MagicSubscriber(
    ros::NodeHandle &ros_node, const std::string subscriber_topic,
    const std::string publisher_topic)
    : nh_{&ros_node}, sub_topic_{subscriber_topic},
      pub_topic_{publisher_topic}, image_sub_{nh_->subscribe(
          sub_topic_, 1, &MagicSubscriber<ROSMessageType>::CallbackToTopic,
          this)},
      image_pub_{nh_->advertise<sensor_msgs::Image>(pub_topic_, 1000)} {

  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Info)) {
    ros::console::notifyLoggerLevelsChanged();
  }
  std::cout << "MagicSubscriber Constructor is called\n";
}

template <typename ROSMessageType>
MagicSubscriber<ROSMessageType>::~MagicSubscriber() {
  std::cout << "MagicSubscriber Destructor is called\n";
}

// callbacks

// generic
template <typename ROSMessageType>
void MagicSubscriber<ROSMessageType>::CallbackToTopic(
    const typename ROSMessageType::ConstPtr &msg) {
  ROS_INFO_STREAM("GENERIC Template Callback message: " << msg);
}

// PointCloud specialization
template <>
inline void MagicSubscriber<sensor_msgs::PointCloud2>::CallbackToTopic(
    const sensor_msgs::PointCloud2::ConstPtr &msg) {
  // the uint8 is an alias of unsigned char, therefore needs casting to int
  ROS_INFO_STREAM("Call Back Topic PC To Depth Image (Data[0] = "
                  << static_cast<int>(msg->data[0]) << ')');

  sensor_msgs::PointCloud point_cloud;

  sensor_msgs::convertPointCloud2ToPointCloud(*msg, point_cloud);

  // These are value sextracted form the rgb camera caminfo
  float K[9] = {343.49636753580074,
                0.0,
                320.5,
                0.0,
                343.49636753580074,
                240.5,
                0.0,
                0.0,
                1.0};
  float centre_x = K[2];
  float centre_y = K[5];
  float focal_x = K[0];
  float focal_y = K[4];
  int cam_info_height = 480;
  int cam_info_width = 640;

  cv::Mat cv_image = cv::Mat(cam_info_height, cam_info_width, CV_32FC1,
                             cv::Scalar(std::numeric_limits<float>::max()));

  for (int i = 0; i < point_cloud.points.size(); i++) {
    if (point_cloud.points[i].z == point_cloud.points[i].z) {
      float z = point_cloud.points[i].z * 1000.0;
      float u = (point_cloud.points[i].x * 1000.0 * focal_x) / z;
      float v = (point_cloud.points[i].y * 1000.0 * focal_y) / z;
      int pixel_pos_x = (int)(u + centre_x);
      int pixel_pos_y = (int)(v + centre_y);

      if (pixel_pos_x > (cam_info_width - 1)) {
        pixel_pos_x = cam_info_width - 1;
      }
      if (pixel_pos_y > (cam_info_height - 1)) {
        pixel_pos_y = cam_info_height - 1;
      }
      cv_image.at<float>(pixel_pos_y, pixel_pos_x) = z;
    }
  }

  putText(cv_image, "Depth CAM", cv::Point(50, 50), cv::FONT_HERSHEY_DUPLEX, 1,
          cv::Scalar(0, 255, 0), 2, false);

  cv_image.convertTo(cv_image, CV_16UC1);

  sensor_msgs::ImagePtr output_image =
      cv_bridge::CvImage(std_msgs::Header(), "16UC1", cv_image).toImageMsg();
  output_image->header.stamp = ros::Time::now();
  image_pub_.publish(output_image);
}

#endif