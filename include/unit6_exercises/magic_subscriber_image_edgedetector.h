#ifndef MAGIC_SUBSCRIBER_H
#define MAGIC_SUBSCRIBER_H

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

static const std::string OPENCV_WINDOW = "Image window";

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
    : sub_topic_{subscriber_topic}, pub_topic_{publisher_topic}, nh_{&ros_node},
      image_sub_{nh_->subscribe(
          this->sub_topic_, 1,
          &MagicSubscriber<ROSMessageType>::CallbackToTopic, this)},
      image_pub_{nh_->advertise<sensor_msgs::Image>(pub_topic_, 1000)}

{
  cv::namedWindow(OPENCV_WINDOW);

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

// specialization Image
template <>
void MagicSubscriber<sensor_msgs::Image>::CallbackToTopic(
    const sensor_msgs::Image::ConstPtr &msg) {
  // the uint8 is an alias of unsigned char, therefore needs casting to int
  ROS_INFO_STREAM(
      "Call Back Topic Image Data[0]=" << static_cast<int>(msg->data[0]));

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // The Edge detection
  // Convert to graycsale

  cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_BGR2GRAY);
  // Blur the image for better edge detection

  GaussianBlur(cv_ptr->image, cv_ptr->image, cv::Size(3, 3), 0);

  // Canny edge detection

  Canny(cv_ptr->image, cv_ptr->image, 100, 200, 3, false);

  putText(cv_ptr->image, "EDGE DETECTION CAM", cvPoint(30, 30),
          cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 250), 1,
          CV_MSA);

  // Display canny edge detected image
  imshow(OPENCV_WINDOW, cv_ptr->image);
  cv::waitKey(3);

  // Update GUI Window
  // imshow(OPENCV_WINDOW, cv_ptr->image);
  // waitKey(3);

  // Output modified video stream
  cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_GRAY2BGR);
  image_pub_.publish(cv_ptr->toImageMsg());
}

#endif