#include "unit6_exercises/magic_subscriber_image_edgedetector.h"
#include <sensor_msgs/Image.h>

using sensor_msgs::Image;

int main(int argc, char **argv) {

  ros::init(argc, argv, "magic_pcl_susbcriber_main_node");

  ros::NodeHandle _n("magic_pcl_susbcriber_main_ns");
  std::string sub_topic_name = "/camera/rgb/image_raw";
  std::string pub_topic_name = "/out/edge_detection";

  MagicSubscriber<Image> magic_subscriber_object(_n, sub_topic_name, pub_topic_name);

  ros::spin();

  return 0;
}