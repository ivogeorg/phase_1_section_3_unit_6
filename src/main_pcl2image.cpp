#include "unit6_exercises/magic_subscriber_pcl2image.h"
#include <sensor_msgs/PointCloud2.h>

using sensor_msgs::PointCloud2;

int main(int argc, char **argv) {

  ros::init(argc, argv, "main_pcl2image_node");

  ros::NodeHandle _n("main_pcl2image_ns");
  std::string sub_topic_name = "/camera/depth_registered/points";
  std::string pub_topic_name = "/out/depth_image";

  MagicSubscriber<PointCloud2> magic_subscriber_object(_n, sub_topic_name, pub_topic_name);

  ros::spin();

  return 0;
}