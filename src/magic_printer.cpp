#include "unit6_exercises/magic_printer.h"

MagicPrinter::MagicPrinter(ros::NodeHandle &ros_node) {

  this->m_ros_node_object = &ros_node;

  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Info)) {
    ros::console::notifyLoggerLevelsChanged();
  }
}

// Destructor
MagicPrinter::~MagicPrinter() {
  cout << "MagicPrinter Destructor is called" << endl;
}