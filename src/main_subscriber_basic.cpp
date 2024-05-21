#include "unit6_exercises/subscriber_basic.h"

int main(int argc, char **argv) {

  ros::init(argc, argv, "magic_susbcriber_main_node");

  ros::NodeHandle _n("magic_susbcriber_main_ns");
  string topic_name = "/camera/rgb/image_raw";

  MagicSubscriber magic_subscriber_object(_n, topic_name);

  ros::spin();

  return 0;
}