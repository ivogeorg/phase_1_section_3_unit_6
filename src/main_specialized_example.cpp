#include "unit6_exercises/specialised_example.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

using sensor_msgs::Image;
using sensor_msgs::PointCloud2;
using std_msgs::String;

int main(int argc, char **argv) {

  ros::init(argc, argv, "magic_susbcriber_main_node");

  ros::NodeHandle _n("magic_susbcriber_main_ns");
  string topic_name_image = "/camera/rgb/image_raw";
  string topic_name_pointcloud = "/camera/depth_registered/points";
  string topic_name = "/dummy_string";

  MagicSubscriber<Image> magic_sub_obj_image(_n, topic_name_image);
  MagicSubscriber<PointCloud2> magic_sub_obj_pcl(_n, topic_name_pointcloud);
  MagicSubscriber<String> magic_sub_obj_generic(_n, topic_name);

  ros::spin();

  return 0;
}