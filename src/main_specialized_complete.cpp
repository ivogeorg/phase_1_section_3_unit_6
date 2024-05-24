#include "unit6_exercises/specialized_complete.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>


using sensor_msgs::Image;
using sensor_msgs::PointCloud2;

int main(int argc, char **argv) {

  ros::init(argc, argv, "main_specialized_complete_node");

  ros::NodeHandle n("magic_susbcriber_complete_nh");

  // Edge detector
  std::string sub_topic_name = "/camera/rgb/image_raw";
  std::string pub_topic_name = "/out/edge_detection";

  MagicSubscriber<Image> edge_detector_object(n, sub_topic_name, pub_topic_name);

  // Depth image
  sub_topic_name = "/camera/depth_registered/points";
  pub_topic_name = "/out/depth_image";

  MagicSubscriber<PointCloud2> depth_image_object(n, sub_topic_name, pub_topic_name);

  ros::spin();

  return 0;
}